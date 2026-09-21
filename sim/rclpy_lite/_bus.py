"""
In-process topic message bus.

Connects publishers and subscribers, enforces QoS compatibility exactly as
real ROS 2 DDS does:
  - BEST_EFFORT publisher + RELIABLE subscriber  → silent no-connection
  - BEST_EFFORT publisher + BEST_EFFORT subscriber → connected
  - RELIABLE publisher  + any subscriber          → connected
"""

import logging
import threading
import traceback
from typing import Any, Callable, Dict, List, Optional

_log = logging.getLogger("rclpy.bus")


class _TopicEntry:
    def __init__(self, msg_type, qos):
        self.msg_type = msg_type
        self.qos = qos
        self.subscribers: List["_SubEntry"] = []
        self.lock = threading.Lock()


class _SubEntry:
    def __init__(self, callback: Callable, qos, node_name: str):
        self.callback = callback
        self.qos = qos
        self.node_name = node_name


class _Bus:
    """Singleton in-process message bus shared across all nodes."""

    _instance: Optional["_Bus"] = None
    _lock = threading.Lock()

    @classmethod
    def instance(cls) -> "_Bus":
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    @classmethod
    def reset(cls) -> None:
        with cls._lock:
            cls._instance = None

    def __init__(self):
        self._topics: Dict[str, _TopicEntry] = {}
        self._lock = threading.RLock()
        self._logger_fn: Callable[[str], None] = lambda msg: None
        # (topic, node_name) -> number of messages delivered. Lets the launcher
        # tell "your node never received anything" apart from "your node ignored it".
        self.delivered: Dict[tuple, int] = {}
        # Exceptions raised inside callbacks: (topic, node_name) -> count
        self.callback_errors: Dict[tuple, int] = {}

    def set_logger(self, fn: Callable[[str], None]) -> None:
        self._logger_fn = fn

    # ------------------------------------------------------------------
    # Publisher side
    # ------------------------------------------------------------------

    def register_publisher(self, topic: str, msg_type, qos) -> None:
        with self._lock:
            if topic not in self._topics:
                self._topics[topic] = _TopicEntry(msg_type, qos)
            else:
                # Update QoS to this publisher's setting (last wins, realistic enough)
                self._topics[topic].qos = qos

    def publish(self, topic: str, msg: Any) -> None:
        with self._lock:
            entry = self._topics.get(topic)
            if entry is None:
                return
            subs = list(entry.subs_snapshot())

        for sub in subs:
            key = (topic, sub.node_name)
            self.delivered[key] = self.delivered.get(key, 0) + 1
            try:
                sub.callback(msg)
            except NotImplementedError:
                # A TODO you have not written yet. Let the launcher explain it.
                raise
            except Exception:
                # Real rclpy prints callback exceptions loudly; swallowing them
                # silently would make bugs in your callback impossible to find.
                n = self.callback_errors.get(key, 0) + 1
                self.callback_errors[key] = n
                if n <= 3:
                    _log.error(
                        f"exception in callback for '{topic}' (node '{sub.node_name}'), "
                        f"occurrence #{n}:\n{traceback.format_exc()}"
                    )
                elif n == 4:
                    _log.error(f"...same callback keeps failing; suppressing further tracebacks for '{topic}'")

    def subscriber_count(self, topic: str, node_name: Optional[str] = None) -> int:
        with self._lock:
            entry = self._topics.get(topic)
            if entry is None:
                return 0
            return sum(1 for s in entry.subscribers if node_name is None or s.node_name == node_name)

    # ------------------------------------------------------------------
    # Subscriber side
    # ------------------------------------------------------------------

    def register_subscriber(
        self,
        topic: str,
        msg_type,
        callback: Callable,
        qos,
        node_name: str,
    ) -> bool:
        """
        Returns True if connected, False if QoS mismatch (silent in log only).
        """
        with self._lock:
            if topic not in self._topics:
                # Publisher not yet registered — store the sub and connect later
                self._topics[topic] = _TopicEntry(msg_type, qos=None)

            entry = self._topics[topic]
            pub_qos = entry.qos

            if pub_qos is not None and not _qos_compatible(pub_qos, qos):
                _log.warning(
                    f"New publisher discovered on topic '{topic}', offering incompatible QoS. "
                    f"No messages will be received from it. Last incompatible policy: RELIABILITY "
                    f"(publisher={pub_qos.reliability.name}, subscriber={qos.reliability.name})"
                )
                return False

            entry.subscribers.append(_SubEntry(callback, qos, node_name))
            return True

    def late_connect_subscribers(self, topic: str, pub_qos) -> None:
        """
        Called when a publisher registers after subscribers already exist.
        Drops any subscribers whose QoS is incompatible.
        """
        with self._lock:
            entry = self._topics.get(topic)
            if entry is None:
                return
            compatible = []
            for sub in entry.subscribers:
                if _qos_compatible(pub_qos, sub.qos):
                    compatible.append(sub)
                else:
                    _log.warning(
                        f"New publisher discovered on topic '{topic}', offering incompatible QoS. "
                        f"No messages will be received from it. Last incompatible policy: RELIABILITY "
                        f"(publisher={pub_qos.reliability.name}, subscriber={sub.qos.reliability.name})"
                    )
            entry.subscribers = compatible


def _qos_compatible(pub_qos, sub_qos) -> bool:
    """
    ROS 2 rule: subscriber reliability must be <= publisher reliability.
    RELIABLE > BEST_EFFORT, so:
      pub=BEST_EFFORT, sub=RELIABLE  → incompatible (sub wants more than pub provides)
      pub=RELIABLE,    sub=RELIABLE  → compatible
      pub=RELIABLE,    sub=BEST_EFFORT → compatible
      pub=BEST_EFFORT, sub=BEST_EFFORT → compatible
    """
    from .qos import ReliabilityPolicy

    if pub_qos.reliability == ReliabilityPolicy.BEST_EFFORT:
        return sub_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    return True


# Patch the _TopicEntry to expose a snapshot method
def _subs_snapshot(self):
    with self.lock:
        return list(self.subscribers)


_TopicEntry.subs_snapshot = _subs_snapshot
