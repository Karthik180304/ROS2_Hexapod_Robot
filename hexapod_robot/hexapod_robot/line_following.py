#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

from hexapod_robot.builder import BuilderX  # Import the BuilderX class

class Linefollower(Node):
    def __init__(self):
        super().__init__('linefollower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/mybot/camera1/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()

        # Create an instance of the BuilderX class
        self.robot = BuilderX()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = 3 * h // 4
        search_bot = 3 * h // 4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            err = cx - w / 2
            self.twist.linear.x = 0.2  # Adjust the linear speed as needed
            self.twist.angular.z = -float(err) / 100  # Adjust the angular speed as needed
            self.cmd_vel_pub.publish(self.twist)

            # Use the BuilderX class to set the robot's walk velocity based on line following
            self.robot.set_walk_velocity(self.twist.linear.x, 0, self.twist.angular.z)
        else:
            # If the line is not detected, stop the robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)

            # Stop the robot using the BuilderX class
            self.robot.set_walk_velocity(0, 0, 0)

        cv2.imshow("mask", mask)
        cv2.imshow("output", image)
        cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    linefollower = Linefollower()
    rclpy.spin(linefollower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
