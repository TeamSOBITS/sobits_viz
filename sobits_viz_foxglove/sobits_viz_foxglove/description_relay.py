# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Republish the robot description with mesh URIs a Foxglove client can fetch."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sobits_viz_foxglove.mesh_uris import rewrite_file_uris
from std_msgs.msg import String


class DescriptionRelay(Node):
    """Rewrite the description's installed file:// mesh URIs as package://."""

    def __init__(self):
        super().__init__('description_relay')
        robot_name = self.declare_parameter('robot_name', '').value
        if not robot_name:
            raise RuntimeError(
                'robot_name is required: the launch file sets it from robot_name:=<robot>')

        source = self._resolve(robot_name, 'input_topic', 'robot_description')
        target = self._resolve(robot_name, 'output_topic', 'robot_description_foxglove')

        # The description is latched, so a viewer that connects later still gets it.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(String, target, latched)
        self._subscription = self.create_subscription(String, source, self._on_urdf, latched)
        self.get_logger().info(f'Relaying {source} to {target}.')

    def _resolve(self, robot_name: str, parameter: str, default: str) -> str:
        topic = self.declare_parameter(parameter, default).value or default
        return topic if topic.startswith('/') else f'/{robot_name}/{topic}'

    def _on_urdf(self, message: String) -> None:
        description, rewritten = rewrite_file_uris(message.data)
        self._publisher.publish(String(data=description))
        self.get_logger().info(
            f'Relayed the robot description: {rewritten} file:// mesh URIs '
            'rewritten to package://.')


def main(args=None):
    """Spin the relay until shutdown."""
    rclpy.init(args=args)
    node = DescriptionRelay()
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
