#!/usr/bin/env python3
"""
vacuum_bridge_node
------------------
Suscribe a /<robot_ns>/vacuum_on_ros (std_msgs/Bool, ROS 2)
y republica en el topic Gazebo Transport /<robot_ns>/vacuum_on
(gz.msgs.Boolean) mediante gz topic.

Si el nodo se lanza en un namespace ROS 2, ese namespace se usa por defecto
para construir ambos topics. Tambien se pueden sobrescribir con parametros.
"""

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class VacuumBridge(Node):
    GZ_MSG    = 'gz.msgs.Boolean'

    def __init__(self):
        super().__init__('vacuum_bridge_node')
        self.declare_parameter('ros_topic', '')
        self.declare_parameter('gz_topic', '')

        namespace = self.get_namespace().strip('/')
        default_ros_topic = f'/{namespace}/vacuum_on_ros' if namespace else '/vacuum_on_ros'
        default_gz_topic = f'/{namespace}/vacuum_on' if namespace else '/vacuum_on'

        self.ros_topic = self.get_parameter('ros_topic').value or default_ros_topic
        self.gz_topic = self.get_parameter('gz_topic').value or default_gz_topic

        self._sub = self.create_subscription(
            Bool, self.ros_topic, self._cb, 10)
        self.get_logger().info(
            f'[VacuumBridge] {self.ros_topic} -> gz {self.gz_topic}')

    def _cb(self, msg: Bool):
        value = 'true' if msg.data else 'false'
        self.get_logger().info(f'[VacuumBridge] vacuum_on={value}')
        try:
            subprocess.run(
                ['gz', 'topic', '-t', self.gz_topic,
                 '-m', self.GZ_MSG, '-p', f'data: {value}'],
                check=True, timeout=2.0)
        except Exception as e:
            self.get_logger().error(f'[VacuumBridge] {e}')


def main():
    rclpy.init()
    node = VacuumBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
