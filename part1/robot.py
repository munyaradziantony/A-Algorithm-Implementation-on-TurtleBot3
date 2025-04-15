from helpers import *
from math import sin, cos, sqrt, pi as PI, radians, degrees
import numpy as np
from transform import *

class Robot:
    def __init__(self, wheel_radius, robot_radius):
        self.dt = 0.1
        self.R = wheel_radius
        self.L = robot_radius
        self.r = self.L / 2
        self.RPM1 = 5
        self.RPM2 = 10
        self.valid_actions = [
            self.sharp_right_R1,
            self.sharp_left_R1,
            self.straight_R1,
            self.sharp_right_R2,
            self.sharp_left_R2,
            self.straight_R2,
            self.gradual_turn_R1R2,
            self.gradual_turn_R2R1,
        ]

    # Function to get the angular velocity from RPM
    def rpm_to_rad_ps(self, RPM):
        return (2 * PI * RPM) / 60

    # Defining set of actions based on wheel RPMs
    def action(self, node, RPM_RW, RPM_LW):
        # Convert the units from RPM to radians/second

        rx,ry,ryaw = transform_map_to_robot(node.x, node.y, node.theta)
        u_l = self.rpm_to_rad_ps(RPM_LW)
        u_r = self.rpm_to_rad_ps(RPM_RW)
        edgecost = 0
        x_i = rx
        y_i = ry
        theta_i = radians(ryaw)

        waypoints:list[WayPoint] = []
        for _ in np.arange(0, 1, 0.1):
            dx = 0.5 * self.R * (u_l + u_r) * cos(theta_i) * self.dt
            dy = 0.5 * self.R * (u_l + u_r) * sin(theta_i) * self.dt
            dtheta = (self.R / self.L) * (u_r - u_l) * self.dt
            dcost = sqrt(dx**2 + dy**2)
            edgecost += dcost
            x_i += dx
            y_i += dy
            theta_i = radians(principal_theta(degrees(theta_i) + degrees(dtheta)))
            tx,ty,tyaw = transform_robot_to_map(x_i, y_i, degrees(theta_i))

            waypoints.append(WayPoint(tx,ty,tyaw, edgecost))
        return waypoints, edgecost

    def sharp_right_R1(self, node):
        return self.action(
            node, self.RPM1, 0
        )  # Pivoting right by applying RPM1 on the left wheel and 0 on the right wheel

    def sharp_left_R1(self, node):
        return self.action(
            node, 0, self.RPM1
        )  # Pivoting left by applying 0 on the left wheel and 0 on the right wheel

    def straight_R1(self, node):
        return self.action(
            node, self.RPM1, self.RPM1
        )  # Move straight by applying RPM1 on both wheels

    def sharp_right_R2(self, node):
        return self.action(
            node, self.RPM2, 0
        )  # Pivoting right by applying RPM2 on the left wheel and 0 on the right wheel

    def sharp_left_R2(self, node):
        return self.action(
            node, 0, self.RPM2
        )  # Pivoting left by applying 0 on the left wheel and RPM2 on the right wheel

    def straight_R2(self, node):
        return self.action(
            node, self.RPM2, self.RPM2
        )  # Move straight by applying RPM2 on both wheels

    def gradual_turn_R1R2(self, node):
        return self.action(
            node, self.RPM1, self.RPM2
        )  # Turn by applying RPM1 on right wheel and RPM2 on left wheel

    def gradual_turn_R2R1(self, node):
        return self.action(
            node, self.RPM2, self.RPM1
        )  # Turn by applying RPM2 on right wheel and RPM1 on left wheel

#
#
# r = Robot(33,287)
# ll,dd = r.action(Node(0,0,90,0),5,5)
#
# for node in ll:
#     print(node)