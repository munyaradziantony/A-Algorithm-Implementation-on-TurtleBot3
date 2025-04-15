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
            clearance = int(input("Enter robot clearance (in pixels): "))
            if clearance < 0:
                print("Warning: Invalid Robot clearance")
                continue
            break
        except ValueError:
            print("Warning: Robot clearance out of bounds. Using default clearance (5 mm)")
            clearance = 5
            break

    return clearance

class A_Star_Path(ROS2Node):

    def __init__(self):
        super().__init__("a_star_path")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def normalize_angle(aself,angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def compute_velocity_to_target(self,A, B, dt):
        """
        A, B: (x, y, theta) - poses in 2D
        dt: time interval to reach from A to B

        Returns:
            linear velocity v (m/s), angular velocity w (rad/s)
        """
        x1, y1, theta1 = A
        x2, y2, theta2 = B

        # Step 1: Relative position in world frame
        dx = x2 - x1
        dy = y2 - y1
        dtheta = normalize_angle(theta2 - theta1)

        # Step 2: Transform dx, dy into A's frame
        # So motion is expressed as if robot is at origin facing theta1
        dx_robot = np.cos(-theta1) * dx - np.sin(-theta1) * dy
        dy_robot = np.sin(-theta1) * dx + np.cos(-theta1) * dy

        # Step 3: Compute linear and angular velocities
        v = np.hypot(dx_robot, dy_robot) / dt
        angle_to_target = np.arctan2(dy_robot, dx_robot)
        angular_offset = normalize_angle(angle_to_target)  # relative to robot forward

        # Adjust linear velocity direction based on angular offset
        v_forward = v * np.cos(angular_offset)
        w = dtheta / dt
        return v_forward, w


    def run_search(self):
        # Create the Robot object
        robot = Robot(33, 287)

        # Prepare the canvas
        cleared = get_clearance()
        canvas = Canvas(WIDTH, HEIGHT, round(cleared + robot.r), MULTIPLIER)

        # Create the search object and pass the robot and canvas objects
        search = Search(robot, canvas)

        if search.a_star():
            print("Path found!")
            search.backtrack_path()
            velocity_message = Twist()
            dt = robot.dt

            for node in search.path:
                # Publish the twist message
                last_waypoint: WayPoint = None

                for i in range(len(node.waypoints) - 1):
                    x0, y0, theta0 = transform_map_to_robot(node.waypoints[i].x, node.waypoints[i].y, node.waypoints[i].theta)
                    x1, y1, theta1 = transform_map_to_robot(node.waypoints[i + 1].x, node.waypoints[i + 1].y, node.waypoints[i + 1].theta)
                    v,w = self.compute_velocity_to_target((x0/1000, y0/1000, radians(theta0)),( x1/1000, y1/1000, radians(theta1)),0.1)
                    velocity_message.linear.x = v
                    velocity_message.angular.z = w
                    self.cmd_vel_pub.publish(velocity_message)
                    time.sleep(dt)

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
