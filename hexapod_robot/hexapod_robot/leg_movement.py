#!/usr/bin/env python3

import rclpy
from threading import Thread
from geometry_msgs.msg import Twist
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from  hexapod_robot.builder import BuilderX


class WFunc:
    """Class for generating walking angles"""
    def __init__(self):
        self.pfn = {
            'j_c1_lf': self.create_function(),
            'j_c1_rm': self.create_function(),
            'j_c1_lr': self.create_function(),
            'j_c1_rf': self.create_function(),
            'j_c1_lm': self.create_function(),
            'j_c1_rr': self.create_function()
        }
        self.afn = {
            'j_c1_lf': self.create_function(),
            'j_c1_rm': self.create_function(),
            'j_c1_lr': self.create_function(),
            'j_c1_rf': self.create_function(),
            'j_c1_lm': self.create_function(),
            'j_c1_rr': self.create_function()
        }
        self.parameters = {
            'vx_scale': 1.0,
            'vt_scale': 1.0
        }

    def create_function(self):
        # Define your custom function here
        # Replace the return statement with your function logic
        return lambda x: 0.0
    
class Walker:
    """Class for making BuilderX walk"""
    def __init__(self, darwin):
        self.darwin = darwin
        self.running = False

        self.velocity = [0, 0, 0]
        self.walking = False
        self.func = WFunc()

        # self.ready_pos=get_walk_angles(10)
        self.ready_pos = self.func.get(True, 0, [0, 0, 0])

        self._th_walk = None

        self._sub_cmd_vel = self.create_subscription(
            Twist,
            darwin.ns + "cmd_vel",
            self._cb_cmd_vel,
            1
        )

    def _cb_cmd_vel(self, msg):
        """Catches cmd_vel and update walker speed"""
        print('cmdvel', msg)
        vx = msg.linear.x
        vy = msg.linear.y
        vt = msg.angular.z
        self.start()
        self.set_velocity(vx, vy, vt)

    def init_walk(self):
        """If not there yet, go to initial walk position"""
        self.get_logger().info('Going to walk position')
        if self.get_dist_to_ready() > 0.02:
            self.darwin.set_angles_slow(self.ready_pos)

    def start(self):
        if not self.running:
            self.running = True
            self.init_walk()
            self._th_walk = Thread(target=self._do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            self.get_logger().info('Waiting for stopped')
            while rclpy.ok() and self._th_walk is not None:
                rclpy.sleep(0.1)
            self.get_logger().info('Stopped')

            self.running = False

    def set_velocity(self, x, y, t):
        self.velocity = [x, y, t]

    def _do_walk(self):
        """Main walking loop
        Smoothly update velocity vectors and apply corresponding angles.
        """
        rate = rclpy.Rate(100)
        self.get_logger().info('Started walking thread')

        func = self.func

        # Global walk loop
        n = 50
        p = True
        i = 0
        self.current_velocity = [0, 0, 0]
        rate = rclpy.Rate(100)
        while (not rclpy.shutdown() and (self.walking or i < n or self.is_walking())):
            if not self.walking:
                self.velocity = [0, 0, 0]
            if not self.is_walking() and i == 0:
                # Do not move if nothing to do and already at 0
                self.update_velocity(self.velocity, n)
                rate.sleep()
                continue
            x = float(i) / n
            angles = self.func.get(p, x, self.current_velocity)
            self.update_velocity(self.velocity, n)
            self.darwin.set_angles(angles)
            i += 1
            if i > n:
                i = 0
                p = not p
            rate.sleep()
        self.get_logger().info('Finished walking thread')

        self._th_walk = None

    def is_walking(self):
        return self.walking

    def update_velocity(self, v, n):
        """Smoothly update velocity vector"""
        dv = [(vi - vf) / float(n) for vi, vf in zip(v, self.current_velocity)]
        self.current_velocity = [vf + dvi for dvi, vf in zip(dv, self.current_velocity)]

    def get_dist_to_ready(self):
        """Return distance to 'ready' position"""
        d = 0
        angles = self.darwin.get_angles()
        for j in self.ready_pos.keys():
            a = angles[j]
            b = self.ready_pos[j]
            d += (a - b) ** 2
        d = d ** 0.5
        return d

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

def main(args=None):
    rclpy.init(args=args)

    darwin = BuilderX()
    walker = Walker(darwin)

    rclpy.spin(walker)

    walker.stop()
    darwin.stop()

    walker.destroy_node()
    darwin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()