#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class BuilderX(Node):

    def __init__(self, ns='/builderx/'):
        super().__init__('builderx')
        self.ns = ns
        self.joints = None
        self.angles = None

        self._sub_joints = self.create_subscription(
            JointState, ns + 'joint_states', self._cb_joints, 1)
        self.get_logger().info('Waiting for joints to be populated...')
        while not self.joints:
            time.sleep(0.1)
            self.get_logger().info('Waiting for joints to be populated...')
        self.get_logger().info('Joints populated')

        self.get_logger().info('Creating joint command publishers')
        self._pub_joints = {}
        for j in self.joints:
            p = self.create_publisher(
                Float64, ns + j + '_position_controller/command', 1)
            self._pub_joints[j] = p

        time.sleep(1)

        self._pub_cmd_vel = self.create_publisher(Twist, ns + 'cmd_vel', 1)

    def set_walk_velocity(self, x, y, t):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t
        self._pub_cmd_vel.publish(msg)

    def _cb_joints(self, msg):
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None or self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self, angles):
        for j, v in angles.items():
            if j not in self.joints:
                self.get_logger().error('Invalid joint name "' + j + '"')
                continue
            msg = Float64()
            msg.data = v
            self._pub_joints[j].publish(msg)

    def set_angles_slow(self, stop_angles, delay=2):
        start_angles = self.get_angles()
        start = time.time()
        stop = start + delay
        r = self.create_rate(100)
        while rclpy.ok():
            t = time.time()
            if t > stop:
                break
            ratio = (t - start) / delay
            angles = interpolate(stop_angles, start_angles, ratio)
            self.set_angles(angles)
            r.sleep()


def interpolate(anglesa, anglesb, coefa):
    z = {}
    joints = anglesa.keys()
    for j in joints:
        z[j] = anglesa[j] * coefa + anglesb[j] * (1 - coefa)
    return z


def get_distance(anglesa, anglesb):
    d = 0
    joints = anglesa.keys()
    if len(joints) == 0:
        return 0
    for j in joints:
        d += abs(anglesb[j] - anglesa[j])
    d /= len(joints)
    return d


def main():
    rclpy.init()
    builderx = BuilderX()
    # Use phantomx object to perform actions or publish messages
    rclpy.spin(builderx)
    builderx.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()