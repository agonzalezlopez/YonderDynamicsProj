"""
The simulator package. Importing it puts this directory on sys.path so that the
shim packages (rclpy_lite, nav_msgs, ...) can be imported by their plain names,
exactly as they would be with a real ROS installation. That is why
`from sim.messages import GridSnapshot` works from anywhere, including tests.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
