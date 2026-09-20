# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Republish /tf at a fixed rate so a viewer's transform buffer lasts."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TransformThrottle(Node):
    """Forward the newest transform per child frame on a timer."""

    def __init__(self):
        super().__init__('transform_throttle')
        source = self.declare_parameter('input_topic', '/tf').value
        target = self.declare_parameter('output_topic', '/tf_throttled').value
        rate = float(self.declare_parameter('rate_hz', 10.0).value)

        self._latest = {}
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(TFMessage, target, qos)
        self._subscription = self.create_subscription(TFMessage, source, self._on_tf, qos)
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'Throttling {source} to {target} at {rate:g} Hz.')

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._latest[transform.child_frame_id] = transform

    def _publish(self) -> None:
        if not self._latest:
            return
        self._publisher.publish(TFMessage(transforms=list(self._latest.values())))
        self._latest.clear()


def main(args=None):
    """Spin the throttle until shutdown."""
    rclpy.init(args=args)
    node = TransformThrottle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
