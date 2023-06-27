#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hexapod_robot.builder import BuilderX


class LegMovementDemo(Node):
    def __init__(self):
        super().__init__('leg_movement_demo')

        self.get_logger().info('Instantiating robot Client')
        self.robot = BuilderX()
        rclpy.sleep(1)

        self.get_logger().info('Leg Movement Demo Starting')

        self.robot.set_walk_velocity(0.2, 0, 0)
        rclpy.sleep(3)
        self.robot.set_walk_velocity(1, 0, 0)
        rclpy.sleep(3)
        self.robot.set_walk_velocity(0, 1, 0)
        rclpy.sleep(3)
        self.robot.set_walk_velocity(0, -1, 0)
        rclpy.sleep(3)
        self.robot.set_walk_velocity(-1, 0, 0)
        rclpy.sleep(3)
        self.robot.set_walk_velocity(1, 1, 0)
        rclpy.sleep(5)
        self.robot.set_walk_velocity(0, 0, 0)

        self.get_logger().info('Leg Movement Demo Finished')


def main(args=None):
    rclpy.init(args=args)
    node = LegMovementDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
