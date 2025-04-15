#!/usr/bin/env python3

import sys
import os
sys.path.append("/home/pranavdm/UMD/Sem-2/ENPM661/Projects/Project-3/Phase2/Git/A-Algorithm-Implementation-on-TurtleBot3/part1")

import rclpy
from rclpy.node import Node as ROS2Node
from geometry_msgs.msg import Twist
from canvas import Canvas
from robot import Robot
from search import Search
from helpers import *
import numpy as np
from math import sqrt, atan2, sin, cos

WIDTH = 5400
HEIGHT = 3000
MULTIPLIER = 0.25


class A_Star_Path(ROS2Node):

    def __init__(self):
        super().__init__("a_star_path")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def run_search(self):
        # Create the Robot object
        robot = Robot(33, 287)

        # Prepare the canvas
        canvas = Canvas(WIDTH, HEIGHT, round(2 + robot.r), MULTIPLIER)

        # Create the search object and pass the robot and canvas objects
        search = Search(robot, canvas)

        if search.a_star():
            self.get_logger().debug("Path found!")
            self.get_logger().debug("Backtracking started")
            search.backtrack_path()
            self.get_logger().debug("Backtracking completed")
            velocity_message = Twist()
            dt = robot.dt

            for node in search.path:
                # Publish the twist message
                last_waypoint: WayPoint = None
                for waypoint in node.waypoints:
                    if last_waypoint is None:
                        last_waypoint = waypoint
                        continue
                    dx = waypoint.x - last_waypoint.x
                    dy = waypoint.y - last_waypoint.y
                    distance = sqrt(dx**2 + dy**2)
                    angle = atan2(dy, dx) - last_waypoint.theta
                    angle = atan2(sin(angle), cos(angle))

                    linear_vel = distance / dt
                    angular_vel = angle / dt
                    velocity_message.linear.x = linear_vel
                    velocity_message.angular.z = angular_vel
                    self.cmd_vel_pub.publish(velocity_message)


def main(args=None):
    rclpy.init(args=args)
    node = A_Star_Path()
    node.run_search()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
