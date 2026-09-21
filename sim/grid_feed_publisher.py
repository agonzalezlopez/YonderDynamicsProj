"""
Grid feed publisher — PROVIDED, do not modify.

Plays the role of the rover's mapping stack. Every tick it:

  1. lets the "sensor" look at the world within SENSOR_RANGE_M of the rover,
  2. updates the rover's belief grid (occupancy + confidence per cell),
  3. publishes the whole grid plus the rover pose as one GridSnapshot on /grid_feed.

Two modes
---------
CLEAN (default — this is Part 1)
    Every cell within sensor range is reported EXACTLY as it is in the world:
    an obstacle is always reported as OCCUPIED, free space always as FREE, with
    confidence 1.0. Cells outside sensor range keep whatever was last observed
    (or UNKNOWN, confidence 0.0, if they never were). Confidence never decays.
    You can ignore the confidence array completely in Part 1.

NOISY (--noise — this is Part 2)
    Same sensor, but imperfect. Read the NOISE table below — every number has
    a comment saying what real-world effect it stands in for.

Publishing QoS is BEST_EFFORT with depth 1 (like real sensor data: a late frame
is worthless, so don't bother retransmitting it). Your subscription must be
compatible with that or you will receive nothing.
"""

import math
import random
from typing import Optional, Tuple

from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from messages import GridSnapshot, UNKNOWN, FREE, OCCUPIED
from world import World

FEED_TOPIC = "/grid_feed"
FEED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

SENSOR_RANGE_M = 8.0
ROBOT_RADIUS_M = 0.5
CONFIDENCE_THRESHOLD = 0.6

# Only used with --noise ----------------------------------------------------
NOISE = {
    # Stereo depth gets worse with distance. Expected confidence of a cell seen at
    # distance d is  1 - RANGE_FALLOFF * (d / SENSOR_RANGE)^2.
    # With 0.75: right next to the rover ≈ 1.0, at the edge of sensor range ≈ 0.25.
    "range_falloff": 0.75,
    # Every individual reading is jittered a little on top of that.
    "confidence_sigma": 0.05,
    # Low confidence is not just a number: it means the reading is more likely to
    # be WRONG. P(cell reported with the opposite value) = err_scale * (1 - conf)^err_power.
    # Wrong readings are phantom obstacles and missed obstacles. Each tick's reading is
    # independent, so a phantom usually vanishes next tick.
    "err_scale": 0.5,
    "err_power": 3,
    # A cell that is out of view (or the sensor is blind) is not re-observed, so the
    # rover trusts it less and less:  confidence *= exp(-dt / decay_tau_s)  each tick.
    "decay_tau_s": 5.0,
}


class GridFeedPublisher(Node):
    def __init__(self, world: World, dt: float, noise: bool = False, seed: int = 0):
        super().__init__("grid_feed_publisher")
        self.world = world
        self.dt = dt
        self.noise = noise
        self.rng = random.Random(seed)
        self.pub = self.create_publisher(GridSnapshot, FEED_TOPIC, FEED_QOS)

        n = world.width * world.height
        self.occupancy = [UNKNOWN] * n
        self.confidence = [0.0] * n
        self.blind = False       # True while a sensor dropout is in progress

    # ── sensor model ────────────────────────────────────────────────────────
    def observe(self, rover_xy: Tuple[float, float], t: float) -> None:
        """Update the belief grid from what the sensor can see right now."""
        w = self.world
        self.blind = self.noise and any(a <= t < b for a, b in w.dropouts)

        if self.noise:
            decay = math.exp(-self.dt / NOISE["decay_tau_s"])
            for i, c in enumerate(self.confidence):
                if c > 0.0:
                    self.confidence[i] = max(0.02, c * decay)
        if self.blind:
            return

        r0, c0 = w.cell_of(*rover_xy)
        k = int(math.ceil(SENSOR_RANGE_M / w.resolution)) + 1
        for r in range(max(0, r0 - k), min(w.height, r0 + k + 1)):
            for c in range(max(0, c0 - k), min(w.width, c0 + k + 1)):
                cx, cy = w.center_of(r, c)
                d = math.hypot(cx - rover_xy[0], cy - rover_xy[1])
                if d > SENSOR_RANGE_M:
                    continue
                truth = bool(w.occupied[r * w.width + c])
                i = r * w.width + c
                if not self.noise:
                    self.occupancy[i] = OCCUPIED if truth else FREE
                    self.confidence[i] = 1.0
                    continue
                q = max(0.02, 1.0 - NOISE["range_falloff"] * (d / SENSOR_RANGE_M) ** 2)
                conf = min(1.0, max(0.02, q + self.rng.gauss(0.0, NOISE["confidence_sigma"])))
                p_err = NOISE["err_scale"] * (1.0 - conf) ** NOISE["err_power"]
                seen = truth != (self.rng.random() < p_err)
                self.occupancy[i] = OCCUPIED if seen else FREE
                self.confidence[i] = conf

    # ── publishing ──────────────────────────────────────────────────────────
    def snapshot(self, rover_xy: Tuple[float, float], t: float, seq: int) -> GridSnapshot:
        w = self.world
        msg = GridSnapshot()
        msg.header.stamp = t
        msg.header.frame_id = "map"
        msg.seq = seq
        msg.resolution = w.resolution
        msg.width = w.width
        msg.height = w.height
        msg.origin_x = w.origin_x
        msg.origin_y = w.origin_y
        # Copies: a real message is serialised, so later changes to the sensor's
        # internal grid must never leak into a message you already received.
        msg.occupancy = list(self.occupancy)
        msg.confidence = list(self.confidence)
        msg.rover_x, msg.rover_y = rover_xy
        msg.goal_x, msg.goal_y = w.goal
        msg.robot_radius_m = ROBOT_RADIUS_M
        msg.sensor_range_m = SENSOR_RANGE_M
        msg.confidence_threshold = CONFIDENCE_THRESHOLD
        return msg

    def publish(self, msg: GridSnapshot) -> None:
        self.pub.publish(msg)
