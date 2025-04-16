#!/usr/bin/env python3

import sys
import os

sys.path.append(
    "/home/pranavdm/UMD/Sem-2/ENPM661/Projects/Project-3/Phase2/Git/A-Algorithm-Implementation-on-TurtleBot3/part1"
)

import rclpy
from rclpy.node import Node as ROS2Node
from geometry_msgs.msg import Twist
from canvas import Canvas
from robot import Robot
from search import Search
from helpers import *
import numpy as np
from math import sqrt, atan2, sin, cos, radians, pi as PI
import time
from scipy.spatial.transform import Rotation as R
from transform import *

WIDTH = 5400
HEIGHT = 3000
MULTIPLIER = 0.25


def normalize_angle(angle):
    """Normalize angle to the range (-pi, pi]."""
    while angle > PI:
        angle -= 2 * PI
    while angle <= -PI:
        angle += 2 * PI
    return angle


def get_clearance():
    """
    Input validation function for radius and clearance.
    Returns: valid radius and clearance

    """
    while True:
        try:
            clearance = int(input("Enter robot clearance (in mm): "))
            if clearance < 0:
                print("Warning: Invalid Robot clearance")
                continue
            break
        except ValueError:
            print(
                "Warning: Robot clearance out of bounds. Using default clearance (5 mm)"
            )
            clearance = 5
            break

    return clearance


class A_Star_Path(ROS2Node):

    def __init__(self):
        super().__init__("a_star_path")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def normalize_angle(aself, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_velocity_to_target(self, lw_radps, rw_radps, R, L):
        # Convert RPM to linear velocities
        v_L = lw_radps * (R / 1000)
        v_R = rw_radps * (R / 1000)

        # Compute robot linear and angular velocities
        v = (v_R + v_L) / 2
        omega = (v_R - v_L) / (L / 1000)

        return v, omega

    def run_search(self):
        # Create the Robot object
        robot = Robot(33, 143.5)

        # Prepare the canvas
        cleared = get_clearance()
        canvas = Canvas(WIDTH, HEIGHT, round(cleared + robot.r), MULTIPLIER)

        # Create the search object and pass the robot and canvas objects
        search = Search(robot, canvas)

        if search.a_star():
            print("Path found!")
            search.backtrack_path()
            velocity_message = Twist()

            for node in search.path:
                # Publish the twist message
                velocity_message.linear.x, velocity_message.angular.z = (
                    self.compute_velocity_to_target(
                        node.actions[0], node.actions[1], robot.R, robot.L
                    )
                )
                self.cmd_vel_pub.publish(velocity_message)
                time.sleep(robot.T)

            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0
            self.cmd_vel_pub.publish(velocity_message)


def main(args=None):
    rclpy.init(args=args)
    node = A_Star_Path()
    node.run_search()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
