#!/usr/bin/env python3
"""
vacuum_bridge_node
------------------
Suscribe a /robot_b/vacuum_on_ros  (std_msgs/Bool, ROS 2)
y republica en el topic Gazebo Transport /robot_b/vacuum_on
(gz.msgs.Boolean) mediante gz topic.

Lanzar con:
  ros2 run vacuum_gripper_plugin vacuum_bridge_node
"""

import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class VacuumBridge(Node):

    ROS_TOPIC = '/robot_b/vacuum_on_ros'
    GZ_TOPIC  = '/robot_b/vacuum_on'
    GZ_MSG    = 'gz.msgs.Boolean'

    def __init__(self):
        super().__init__('vacuum_bridge_node')
        self._sub = self.create_subscription(
            Bool, self.ROS_TOPIC, self._cb, 10)
        self.get_logger().info(
            f'[VacuumBridge] {self.ROS_TOPIC} -> gz {self.GZ_TOPIC}')

    def _cb(self, msg: Bool):
        value = 'true' if msg.data else 'false'
        self.get_logger().info(f'[VacuumBridge] vacuum_on={value}')
        try:
            subprocess.run(
                ['gz', 'topic', '-t', self.GZ_TOPIC,
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
