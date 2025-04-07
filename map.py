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

# Set the size of the canvas
WIDTH, HEIGHT = 5400, 3000

# Scale factor for resizing
SCALE = 3

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

# Clearance BUFFER
BUFFER = robot_radius + clearance
GOAL_THRESHOLD = 1.5

# Function to initialize the canvas and map
def initialize_map():
    canvas = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    map = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)  # 3 channels for RGB colors
    return canvas, map

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
def display_resized_map(canvas):
    small_canvas = cv2.resize(canvas, (WIDTH // SCALE, HEIGHT // SCALE), interpolation=cv2.INTER_AREA)
    cv2.imshow(f"Map Scaled {SCALE} times smaller", small_canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Function to save the map image
def save_map_image(small_canvas):
    cv2.imwrite('scaled_map_with_BUFFER_output.png', small_canvas)

# Main function to run the program
def main():
    canvas, map = initialize_map()
    update_map(map)
    canvas[:, :] = map  # Transfer map to canvas
    display_resized_map(canvas)
    save_map_image(canvas)

# Run the program
if __name__ == "__main__":
    main()




# initial_x =0
# initial_y =0
# goal_node = 0
# initial_theta =0
# RPM1 = 0
# RPM2 = 0


# # Cost to come using Euclidean distance
# def get_c2c(x,y):
#     goal_x,goal_y = goal_node
#     dist = math.sqrt(((goal_x-x)**2)+((goal_y-y)**2))
#     return dist

# # Variable for starting node
# initial_c2c = get_c2c(initial_x,initial_y)

# # Defining the initial/starting node
# initial_node = [(initial_x , initial_y , initial_theta , 0 , initial_c2c) , [(0,0,0)], []]

# # Total Cost for initial/starting node
# initial_total_cost = initial_c2c + 0

# # Function to get the angular velocity from RPM
# def get_angular_vel(RPM):
#     return (2*math.pi*RPM)/60

# # Function to get the new x, y and theta values after action
# def get_x_y_theta(RPM_right,RPM_left,x,y,theta):
#     # Extracting wheel angular velocities
#     av_right=get_angular_vel(RPM_right)
#     av_left=get_angular_vel(RPM_left)
    
#     # Initially defining new x as x , new y as y and new theta as theta
#     new_x=x
#     new_y=y
#     new_theta=theta    
#     count=0   
#     # Executing the loop 20 times at a time step of 0.05 to represent a time of 1 second (the action is executed for 1 second)
#     while count<20:
#         dx=(wheel_radius/2)*(av_right+av_left)*math.cos(math.radians(new_theta))*0.05
#         dy=(wheel_radius/2)*(av_right+av_left)*math.sin(math.radians(new_theta))*0.05
#         dtheta=(wheel_radius/wheel_dist)*(av_right-av_left)*0.05
#         new_x=new_x+dx
#         new_y=new_y+dy
#         new_theta=new_theta+math.degrees(dtheta)
#         count=count+1
#     return new_x,new_y,new_theta

# # Defining set of actions based on wheel velocities / RPMs

# # turn right
# def move1(node):
#     v1=0 #right
#     v2=RPM1 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history] 

#     return new_node

# def move2(node):
#     v1=RPM1 #right
#     v2=0 # left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]
    
#     return new_node
    
# def move3(node):
#     v1=RPM1 #right
#     v2=RPM1 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]
    
#     return new_node

# def move4(node):
#     v1=0 #right
#     v2=RPM2 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]

#     return new_node

# def move5(node):
#     v1=RPM2 #right
#     v2=0 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]

#     return new_node

# def move6(node):
#     v1=RPM2 #right
#     v2=RPM2 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]

#     return new_node

# def move7(node):
#     v1=RPM1 #right
#     v2=RPM2 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]

#     return new_node

# def move8(node):
#     v1=RPM2 #right
#     v2=RPM1 #left
#     (x,y,theta,c2c,_)=node[0]
#     path = node[1].copy() # Path for backtracking
#     action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
#     path.append((x, y, theta))
#     action_history.append((v1,v2))
#     new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
#     new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
#     new_c2c = get_c2c(new_x,new_y)
#     new_node=[(new_x,new_y,new_theta,new_c2c,new_c2c),path,action_history]

#     return new_node

