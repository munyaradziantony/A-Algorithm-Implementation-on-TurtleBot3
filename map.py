#! /usr/bin/python3

import heapq
import math
import threading
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt

# A container to hold multiple time values in key: value pair to be used for analysis at the termination of program
time_dict = {}

# Set the size of the map
WIDTH, HEIGHT = 5400, 3000

# # Scale factor for resizing
# SCALE = 3

# Colors (BGR format for OpenCV)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
BLUE = (255, 0, 0)
NEON_GREEN = (20, 255, 57)
NAVY_BLUE = (42, 27, 13)
SILVER_GRAY = (176, 176, 176)
BLACK = (0, 0, 0)

# Robot variables
robot_radius = 80
wheel_radius = 25
wheel_dist = 160
clearance = 100
RPM1 = 0
RPM2 = 0

# Clearance BUFFER
BUFFER = robot_radius + clearance
GOAL_THRESHOLD = 1.5

def get_clearance():
    while True:
        try:
            clearance = int(input("Enter the robot clearance (must be an integer > 0): "))
            if clearance > 0:
                return clearance
            else:
                print("Clearance must be greater than 0.")
        except ValueError:
            print("Invalid input. Please enter a valid integer.")
        
            
#scale use input and validation
def get_scale_factor():
    while True:
        try:
            SCALE = int(input("Enter a scale factor (must be an integer ≥ 1): "))
            if SCALE >= 1:
                return SCALE
            else:
                print("Scale factor must be greater than or equal to 1.")
        except ValueError:
            print("Invalid input. Please enter a valid integer.")

# Function to initialize the map and map
def initialize_map():
    map = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    return map

# Function to check if a point is inside an obstacle
def is_inside_obstacle(x, y):
    obstacle_upper_left = (2100 <= x <= 2200 and 0 <= y <= 2000)
    obstacle_upper_right = (3200 <= x <= 3300 and 0 <= y <= 1000)
    obstacle_bottom_left = (1000 <= x <= 1100 and 1000 <= y <= 3000)
    obstacle_bottom_centre = (3200 <= x <= 3300 and 2000 <= y <= 3000)
    obstacle_bottom_right = (4300 <= x <= 4400 and 1000 <= y <= 3000)

    return any([obstacle_upper_left, obstacle_upper_right, obstacle_bottom_left,
                obstacle_bottom_centre, obstacle_bottom_right])

# Function to check if a point is inside a buffer zone
def is_inside_buffer(x, y):
    obstacle_upper_left_BUFFER = (2100 - BUFFER <= x <= 2200 + BUFFER and 0 <= y <= 2000 + BUFFER)
    obstacle_upper_right_BUFFER = (3200 - BUFFER <= x <= 3300 + BUFFER and 0 <= y <= 1000 + BUFFER)
    obstacle_bottom_left_BUFFER = (1000 - BUFFER <= x <= 1100 + BUFFER and 1000 - BUFFER <= y <= 3000)
    obstacle_bottom_centre_BUFFER = (3200 - BUFFER <= x <= 3300 + BUFFER and 2000 - BUFFER <= y <= 3000)
    obstacle_bottom_right_BUFFER = (4300 - BUFFER <= x <= 4400 + BUFFER and 1000 - BUFFER <= y <= 3000)
    
    x_left_boundary = (0 <= x < BUFFER)
    x_right_boundary = (WIDTH-BUFFER <= x < WIDTH)
    y_upper_boundary = (0 <= y < BUFFER)
    y_lower_boundary = (HEIGHT - BUFFER <= y < HEIGHT)

    return any([obstacle_upper_left_BUFFER, obstacle_upper_right_BUFFER, obstacle_bottom_left_BUFFER,
                obstacle_bottom_centre_BUFFER, obstacle_bottom_right_BUFFER, x_left_boundary,
                x_right_boundary, y_upper_boundary, y_lower_boundary])

# Function to update the map based on obstacles and buffer zones
def update_map(map):
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if is_inside_obstacle(x, y):
                map[y, x] = SILVER_GRAY  # Assign SILVER_GRAY for obstacles
            elif is_inside_buffer(x, y):
                map[y, x] = NEON_GREEN  # Assign NEON_GREEN for clearance
            
# Function to resize and display the map
def display_resized_map(map):
    small_map = cv2.resize(map, (WIDTH // SCALE, HEIGHT // SCALE), interpolation=cv2.INTER_AREA)
    cv2.imshow(f"Map Scaled {SCALE} times smaller", small_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Function to save the map image
def save_map_image(map, SCALE):
    small_map = cv2.resize(map, (WIDTH // SCALE, HEIGHT // SCALE), interpolation=cv2.INTER_AREA)
    cv2.imwrite('scaled_map_with_BUFFER_output.png', small_map)
  
def get_start_position():
    print("Enter Start Coordinates (x y):")
    while True:
        try:
            x_s = int(input(f"Enter start x (0 to {WIDTH - 1}): "))
            y_s = int(input(f"Enter start y (0 to {HEIGHT - 1}): "))

            if not (0 <= x_s < WIDTH and 0 <= y_s < HEIGHT):
                print("Coordinates are out of bounds.")
                continue

            if is_inside_obstacle(x_s, y_s):
                print("Start position is inside an obstacle. Try again.")
                continue

            if is_inside_buffer(x_s, y_s):
                print("Start position is too close to an obstacle (in buffer zone). Try again.")
                continue

            return (x_s, y_s)

        except ValueError:
            print("Invalid input. Please enter integers for both x and y.")
            
def get_valid_start_orientation():
    """
    Input validation function for start orientation.
    Returns: Valid start orientation

    """    
    while True:
        try:
            theta_s = float(input("Enter start orientation in degrees (-180 ≤ θ < 180): "))
            if -180 <= theta_s < 180:
                return theta_s
            else:
                print("Orientation must be between -180 (inclusive) and 180 (exclusive).")
                continue
        except ValueError:
            print("Invalid input. Please enter a number.")
            
def get_goal_position():
    print("Enter Goal Coordinates (x y):")
    while True:
        try:
            x_g = int(input(f"Enter goal x (0 to {WIDTH - 1}): "))
            y_g = int(input(f"Enter goal y (0 to {HEIGHT - 1}): "))

            if not (0 <= x_g < WIDTH and 0 <= y_g < HEIGHT):
                print("Coordinates are out of bounds.")
                continue

            if is_inside_obstacle(x_g, y_g):
                print("Goal position is inside an obstacle. Try again.")
                continue

            if is_inside_buffer(x_g, y_g):
                print("Goal position is too close to an obstacle (in buffer zone). Try again.")
                continue

            return (x_g, y_g)

        except ValueError:
            print("Invalid input. Please enter integers for both x and y.")

def get_wheel_rpms():
    while True:
        try:
            RPM1 = float(input("Enter RPM1 (must be > 0): "))
            RPM2 = float(input("Enter RPM2 (must be > 0): "))

            if RPM1 > 0 and RPM2 > 0:
                return (RPM1, RPM2)
            else:
                print("Both RPM values must be greater than 0.")

        except ValueError:
            print("Invalid input. Please enter numeric values for RPMs.")
            
# Point container class
class Point:
    """
    Point class to represent a point in a 2D space. This data structure is also used in as key in dict so the thresholding happens here
    """

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = principal_theta(theta)

    def move(self, dx, dy, dtheta):
        self.x += dx
        self.y += dy
        self.theta += dtheta

    def __hash__(self):
        """Quantize x and y to a 0.5 unit grid"""
        quantized_x = round(self.x * 2) / 2  # Round to nearest 0.5
        quantized_y = round(self.y * 2) / 2

        # Quantize theta to bins of 30 degrees
        quantized_theta = round(self.theta / 30) * 30

        return hash((quantized_x, quantized_y, quantized_theta))

    def __eq__(self, other):
        """ Two nodes are considered equal if they are within 0.5 units and their theta difference is ≤ 30 degrees. """
        if isinstance(other, Point):
            euclidean_distance = math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(theta_diff, 360 - theta_diff)  # Normalize to range [0, 180]

            return euclidean_distance <= 0.5 and theta_diff <= 30
        return False

    def __str__(self):
        """utility function to print Point directly"""
        return f"Point({self.x}, {self.y}, {self.theta})"


# Node container class
def principal_theta(theta):
    """
    Function to find principal theta of a given theta i.e. theta should always be in (-180,180]
    Args:
        theta: theta to be converted

    Returns: normalised theta

    """
    theta = (theta + 180) % 360 - 180  # Ensures theta is in (-180, 180]
    return theta


class Node:
    """
    A node structure to be used in graph. It represents a valid configuration of robot in search graph
    """

    def __init__(self, x, y, theta, c2c):
        self.x = x
        self.y = y
        self.theta = principal_theta(theta)
        self.c2c = c2c
        self.total_cost = 0
        self.parent = None
        self.visited = False

    # used for duplicate key finding in dict
    def __hash__(self):
        """Quantize x and y to a 0.5 unit grid and multiple of 30 degrees"""
        quantized_x = round(self.x * 2) / 2  # Round to nearest 0.5
        quantized_y = round(self.y * 2) / 2

        # Quantize theta to bins of 30 degrees
        quantized_theta = round(self.theta / 30) * 30

        return hash((quantized_x, quantized_y, quantized_theta))

    def __eq__(self, other):
        """ Two nodes are considered equal if they are within 0.5 units and their theta difference is ≤ 30 degrees. """
        if isinstance(other, Node):
            euclidean_distance = math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(theta_diff, 360 - theta_diff)  # Normalize to range [0, 180]

            return euclidean_distance <= 0.5 and theta_diff <= 30
        return False

    def __lt__(self, other):
        return self.total_cost < other.total_cost

    def __str__(self):
        """Utility function to print Node directly"""
        return f"Node(x={self.x}, y={self.y}, θ={self.theta}, c2c={self.c2c})"


#         Container to store goal
class GoalPt:
    """
    Goal pt class to hold X Y Radius of goal point
    """

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

 #==========================================================================================
initial_x = x_s
initial_y =y_s
goal_node = 0
initial_theta = theta_s
RPM1 = 0
RPM2 = 0 

# Cost to come using Euclidean distance
def get_c2c(x,y):
    goal_x,goal_y = goal_node
    dist = math.sqrt(((goal_x-x)**2)+((goal_y-y)**2))
    return dist

# Variable for starting node
initial_c2c = get_c2c(initial_x,initial_y)

# Defining the starting node
initial_node = [(initial_x , initial_y , initial_theta , 0 , initial_c2c) , [(0,0,0)], []]

# Total Cost for the starting node
initial_total_cost = initial_c2c + 0

# Function to get the angular velocity from RPM
def get_angular_vel(RPM):
    return (2*math.pi*RPM)/60

# Function to get the new x, y and theta values after action
def get_x_y_theta(RPM_right,RPM_left,x,y,theta):
    # Extracting wheel angular velocities
    average_velocity_right=get_angular_vel(RPM_right)
    average_velocity_left=get_angular_vel(RPM_left)
    
    # Initially defining new x as x , new y as y and new theta as theta
    new_x=x
    new_y=y
    new_theta=theta    
    count=0   
    # Looping 10 times at a time step of 0.1 to represent a time of 1 second
    while count<100:
        dx=(wheel_radius/2)*(average_velocity_right+average_velocity_left)*math.cos(math.radians(new_theta))*0.1
        dy=(wheel_radius/2)*(average_velocity_right+average_velocity_left)*math.sin(math.radians(new_theta))*0.1
        dtheta=(wheel_radius/wheel_dist)*(average_velocity_right-average_velocity_left)*0.1
        new_x=new_x+dx
        new_y=new_y+dy
        new_theta=new_theta+math.degrees(dtheta)
        count=count+1
    return new_x,new_y,new_theta

# Defining set of actions based on wheel velocities / RPMs
# turn right
def sharp_right_slow(node):
    v1=0 #right
    v2=RPM1 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs] 

    return new_node

def sharp_left_slow(node):
    v1=RPM1 #right
    v2=0 # left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]
    
    return new_node
    
def straight_slow(node):
    v1=RPM1 #right
    v2=RPM1 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]
    
    return new_node

def sharper_right(node):
    v1=0 #right
    v2=RPM2 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]

    return new_node

def sharper_left(node):
    v1=RPM2 #right
    v2=0 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]

    return new_node

def straight_fast(node):
    v1=RPM2 #right
    v2=RPM2 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy()
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]

    return new_node

def gradual_right(node):
    v1=RPM1 #right
    v2=RPM2 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]

    return new_node

def gradual_left(node):
    v1=RPM2 #right
    v2=RPM1 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() 
    action_logs=node[2].copy() 
    path.append((x, y, theta))
    action_logs.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2c = get_c2c(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_logs]

    return new_node



# Main function to run the program
def main():
    global SCALE, clearance, BUFFER

    SCALE = get_scale_factor()
    clearance = get_clearance()
    BUFFER = robot_radius + clearance

    map = initialize_map()
    update_map(map)

    start_position = get_start_position()
    start_orientation = get_valid_start_orientation()
    goal_position = get_goal_position()
    rpm1, rpm2 = get_wheel_rpms()

    print(f"Start position: {start_position}")
    print(f"Start orientation: {start_orientation}°")
    print(f"Goal position: {goal_position}")
    print(f"Wheel RPMs: RPM1 = {RPM1}, RPM2 = {RPM2}")

    display_resized_map(map)
    save_map_image(map, SCALE)


# Run the program
if __name__ == "__main__":
    main()



