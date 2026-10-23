"""ROS 2 simulation-time clock controller."""

from __future__ import annotations

from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class ClockController:
    def __init__(self, node: Node, enabled: bool = True, topic: str = '/clock'):
        self.node = node
        self.enabled = enabled
        self.topic = topic
        self.publisher = node.create_publisher(Clock, topic, 10)
        self.current_time = 0.0

    def advance_to(self, seconds: float) -> None:
        if not self.enabled:
            return
        self.current_time = float(seconds)
        msg = Clock()
        msg.clock.sec = int(self.current_time)
        msg.clock.nanosec = int((self.current_time - int(self.current_time)) * 1_000_000_000)
        self.publisher.publish(msg)
