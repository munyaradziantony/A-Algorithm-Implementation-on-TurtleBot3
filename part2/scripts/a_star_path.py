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


WIDTH = 5400
HEIGHT = 3000
MULTIPLIER = 0.25


def transform_map_to_robot(x, y, yaw):
    # ---- Pose in Frame A ----
    position_A = np.array([x, y, 0.0])
    orientation_A = R.from_euler("xyz", [0, 0, yaw], degrees=False).as_matrix()

    # Homogeneous transformation matrix of pose in A
    T_pose_in_A = np.eye(4)
    T_pose_in_A[:3, :3] = orientation_A
    T_pose_in_A[:3, 3] = position_A

    # ---- Transform from Frame A to Frame B ----
    # Example: Frame B is rotated and translated w.r.t Frame A
    rotation_AB = R.from_euler("xyz", [PI, 0, 0], degrees=False).as_matrix()
    translation_AB = np.array([0, -HEIGHT / 2, 0])

    T_AB = np.eye(4)
    T_AB[:3, :3] = rotation_AB
    T_AB[:3, 3] = translation_AB

    # Now transform the pose from frame A to frame B
    T_pose_in_B = np.linalg.inv(T_AB) @ T_pose_in_A

    # Extract the new position and orientation
    position_B = T_pose_in_B[:3, 3]
    orientation_B = R.from_matrix(T_pose_in_B[:3, :3]).as_euler("xyz", degrees=True)

    return position_B[0], position_B[1], orientation_B[2]


def normalize_angle(angle):
    """Normalize angle to the range (-pi, pi]."""
    while angle > PI:
        angle -= 2 * PI
    while angle <= -PI:
        angle += 2 * PI
    return angle

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
            print("Path found!")
            search.backtrack_path()
            velocity_message = Twist()
            dt = robot.dt

            for node in search.path:
                # Publish the twist message
                last_waypoint: WayPoint = None

                for i in range(len(node.waypoints) - 1):
                    x0, y0, theta0 = node.waypoints[i].x, node.waypoints[i].y, node.waypoints[i].theta
                    x1, y1, theta1 = node.waypoints[i + 1].x, node.waypoints[i + 1].y, node.waypoints[i + 1].theta

                    dx = x1 - x0
                    dy = y1 - y0
                    ds = sqrt(dx**2 + dy**2)
                    
                    dtheta = normalize_angle(theta1 - theta0)

                    linear_vel = ds / (dt * 1000)
                    angular_vel = dtheta / dt
                    velocity_message.linear.x = linear_vel
                    velocity_message.angular.z = angular_vel
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
