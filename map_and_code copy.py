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
WIDTH, HEIGHT = 5400 , 3000

# Scale factor for resizing
SCALE = 4

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
clearance = 80

# Clearance BUFFER
BUFFER = robot_radius + clearance
GOAL_THRESHOLD = 1.5

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

    def __hash__(self):
        """
        Hash function to store nodes uniquely based on quantized x, y, and theta.
        """
        quantized_x = round(self.x * 2) / 2
        quantized_y = round(self.y * 2) / 2
        quantized_theta = round(self.theta / 30) * 30
        return hash((quantized_x, quantized_y, quantized_theta))

    def __eq__(self, other):
        """
        Equality check based on quantized position and orientation.
        """
        if isinstance(other, Node):
            euclidean_distance = math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(theta_diff, 360 - theta_diff)
            return euclidean_distance <= 0.5 and theta_diff <= 30
        return False


    def __lt__(self, other):
        return self.total_cost < other.total_cost

    def __str__(self):
        """Utility function to print Node directly"""
        return f"Node(x={self.x}, y={self.y}, θ={self.theta}, c2c={self.c2c})"


# Container to store goal
class GoalPt:
    """
    Goal pt class to hold X Y Radius of goal point
    """

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

# ======================================= MAP DEFINITIONS =======================================#
#Function to initialize the canvas and map
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

def draw_map():
    # Initialize the map and canvas
    canvas, map = initialize_map()
    
    # Update the map with obstacles and buffer zones
    update_map(map)
    
    # Resize the canvas for display
    small_canvas = cv2.resize(canvas, (WIDTH // SCALE, HEIGHT // SCALE), interpolation=cv2.INTER_AREA)
    
    # Display the map
    cv2.imshow(f"Map Scaled {SCALE} times smaller", small_canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Save the map image
    cv2.imwrite('scaled_map_with_BUFFER_output.png', small_canvas)

# ===================================== A_STAR ALGORITHM =====================================#

def heuristic(node: Point, goal: Point):
    """
    Heuristic function for A*. Since C-space is R^2 using Euclidean distance
    Args:
        node: point 1
        goal: point 2

    Returns: Euclidean distance between node and goal

    """
    return math.hypot(goal.x - node.x, goal.y - node.y)

class Search:
    """
    Search class encapsulating the A* algorithm.
    """

    def __init__(self, radius, clearance, step):
        self.nodes_dict = {}
        self.queue = []
        self.search_goal = None
        self.search_start = None
        self.search_last_node = None
        self.rad = radius
        self.clear = clearance
        self.step = step  # Pass STEP as an argument


    def reached_goal(self, x, y, goal: GoalPt):
        """
        Termination condition if robot reached the goal region, In this Project ignoring the goal theta  so basically robot can be in any orientation in goal region
        :param x: Current X coordinate
        :param y: Current Y coordinate
        :param goal: Goal region (X,Y,Radius)
        :return: Boolean
        """
        if (x - goal.x) ** 2 + (y - goal.y) ** 2 < goal.radius ** 2:
            return True
        else:
            return False

    def a_star(self, start: Point, goal: GoalPt):
        start_time = time.perf_counter()
        self.search_start = start
        self.search_goal = Point(goal.x, goal.y, 0)
        self.nodes_dict.clear()

        # Priority queue initialization
        u = Node(start.x, start.y, start.theta, 0)
        heapq.heappush(self.queue, u)
        self.nodes_dict[Point(u.x, u.y, u.theta)] = u
        goal_reached = False

        while self.queue:
            node = heapq.heappop(self.queue)
            
            # Check if we reached the goal
            if self.reached_goal(node.x, node.y, goal):
                self.nodes_dict["last"] = node
                self.search_last_node = node
                goal_reached = True
                print("Path found!")
                break

            for dx, dy, dtheta, cost in valid_actions:
                next_node = self.nodes_dict.get(Point(node.x + dx, node.y + dy, node.theta + dtheta),
                                                Node(node.x + dx, node.y + dy, node.theta + dtheta, 0))
                
                # Ensure move is in bounds and not in an obstacle
                if 0 <= next_node.x < WIDTH and 0 <= next_node.y < HEIGHT and not is_inside_obstacle(int(next_node.x), int(next_node.y)):
                    # Only revisit if the new path is better
                    if next_node.c2c > node.c2c + cost or not next_node.visited:
                        next_node.c2c = node.c2c + cost
                        next_node.total_cost = next_node.c2c + heuristic(Point(next_node.x, next_node.y, 0), self.search_goal)
                        next_node.visited = True
                        next_node.parent = node
                        heapq.heappush(self.queue, next_node)
                        self.nodes_dict[Point(next_node.x, next_node.y, next_node.theta)] = next_node

        if not goal_reached:
            print("No path found!")
            return False

        end_time = time.perf_counter()
        time_dict["ASTAR"] = end_time - start_time
        return True


    def backtrack_path(self):
        start_time = time.perf_counter()

        path = []
        g = self.nodes_dict.get("last")
        while g is not None:
            path.append(g)
            g = g.parent  # Backtrack using parent pointers

        path.reverse()
        end_time = time.perf_counter()
        time_dict["Backtracking"] = end_time - start_time
        return path


    def draw_explored(self):
        """
        Visualizes the explored nodes of graph.
        """
        cv2.circle(canvas, (self.search_goal.x, self.search_goal.y), 15, BLUE, 2)
        cv2.circle(canvas, (self.search_start.x, self.search_start.y), 5, RED, 2)
        for node in self.nodes_dict:
            cv2.circle(canvas, (node.x, node.y), 1, RED, -1)

        # Show the updated canvas with OpenCV
        cv2.imshow("Explored Nodes", canvas)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
def create_vector(point1: Node, point2: Node):
    """
    Creates and plots a vector between two points.

    Args:
    point1: Node 1
    point2: Node 2
    """
    x1 = point1.x
    y1 = HEIGHT - point1.y  # Flip y-coordinate for OpenCV (origin is at top-left corner)
    x2 = point2.x
    y2 = HEIGHT - point2.y  # Same here

    # Calculate the vector components
    dx = x2 - x1
    dy = y2 - y1

    # Plot the vector using OpenCV's line function
    # Using an arrow for better visualization
    cv2.arrowedLine(canvas, (x1, y1), (x2, y2), (0, 0, 255), 2, tipLength=0.05)

def draw_path_arrows(path):
    """
    Draws arrows along the path.
    
    Args:
        path: List of points (Nodes) to be connected by arrows.
    """
    prev_point = None
    for point in path:
        if prev_point is not None:
            create_vector(prev_point, point)
        prev_point = point

    # Display the result using OpenCV
    cv2.imshow("Path with Arrows", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def draw_path(path):
    """
    Visualizes the optimal path by drawing small circles for each path point.

    Args:
        path: List of points (Nodes) to be visualized.
    """
    for point in path:
        # Draw a small circle at each path point
        cv2.circle(canvas, (point.x, HEIGHT - point.y), 2, (255, 0, 0), -1)  # Blue circles for path points

    # Display the result using OpenCV after drawing everything
    cv2.imshow("Optimal Path", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("Optimal path found and displayed!")
# ===================================== USER INPUT VALIDATION =====================================#
def get_radius_and_clearance():
    """
    Input validation function for radius and clearance.
    Returns: valid radius and clearance

    """
    while True:
        try:
            radius = int(input("Enter robot radius (in pixels): "))
            if not (0 < radius <= 80):
                print("Warning: Robot radius out of bounds. Using default radius (80 mm)")
                radius = 80
            break
        except ValueError:
            print("Warning: Robot radius out of bounds. Using default radius (80 mm)")
            radius = 80
            break

    while True:
        try:
            clearance = int(input("Enter robot clearance (in pixels): "))
            if not (0 <= clearance <= 100):
                print("Warning: Robot clearance out of bounds. Using default clearance (100 mm)")
                continue
            break
        except ValueError:
            print("Warning: Robot clearance out of bounds. Using default clearance (100 mm)")
            clearance = 100
            break

    return radius, clearance


def get_valid_start_coordinates(radius, clearance):
    """
    Input validation function for start coordinates.
    Args:
        radius:
        clearance:

    Returns: Valid start coordinates

    """
    print("Enter Start Coordinates (x y):")
    while True:
        try:
            x_s, y_s = map(int, input("Start (x y ) e.g 300 300: ").split())
            if not (0 <= x_s < WIDTH and 0 <= y_s < HEIGHT):
                print("Error: Start point out of bounds.")
                continue
            if is_inside_obstacle(x_s, HEIGHT - y_s):
                print("Error: Start point is inside an obstacle.")
                continue
            break
        except ValueError:
            print("Error: Enter two integers separated by a space.")
    return x_s, HEIGHT - y_s


def get_valid_start_orientation():
    """
    Input validation function for start orientation.
    Returns: Valid start orientation

    """
    while True:
        try:
            theta = int(input("Start orientation θ (as a multiple of 30 degrees): "))
            if theta % 30 != 0:
                print("Error: Start orientation must be an integer multiple of 30.")
                continue
            if theta < -180 or theta > 180:
                print("Error: Start orientation exceeds limits")
                continue
            return theta
        except ValueError:
            print("Error: Invalid input! Enter an integer (e.g., 0, 30, -30, 60...).")


def get_valid_goal_coordinates(radius, clearance):
    """
    Input validation function for goal coordinates.
    Args:
        radius:
        clearance:

    Returns: Valid goal coordinates

    """
    print("Enter Goal Coordinates (x y):")
    while True:
        try:
            x_g, y_g = map(int, input("Goal (x y) e.g 500 2500 : ").split())
            if not (0 <= x_g < WIDTH and 0 <= y_g < HEIGHT):
                print("Error: Goal point out of bounds.")
                continue
            if is_inside_obstacle(x_g, HEIGHT - y_g):
                print("Error: Goal point is inside an obstacle.")
                continue
            break
        except ValueError:
            print("Error: Enter two integers separated by a space.")
    return x_g, HEIGHT - y_g


def get_valid_goal_orientation():
    """
    Input validation function for goal orientation.
    Returns: Valid goal orientation

    """
    while True:
        try:
            theta = int(input("Goal orientation θ (as a multiple of 30 degrees): "))
            if theta % 30 != 0:
                print("Error: Goal orientation must be an integer multiple of 30.")
                continue
            if theta < -180 or theta > 180:
                print("Error: Goal orientation exceeds limits")
                continue
            return theta
        except ValueError:
            print("Error: Invalid input! Enter an integer (e.g., 0, 30, -30, 60...).")


def get_valid_step_size():
    """
    Input validation function for step size.
    Returns: Valid step size
    """
    while True:
        try:
            step = int(input("Enter robot step size (1 to 10): "))
            if 1 <= step <= 10:
                return step
            else:
                print("Step size must be between 1 and 10.")
        except ValueError:
            print("Enter a valid integer step size.")

# ===================================== MAIN FUNCTION =====================================#

def main():
    """
    Main function for running the A* algorithm and visualizing the results using OpenCV.
    """
    # Request user inputs, check for validity. If invalid, repeat until valid coordinates are given
    # Get clearance and radius
    radius, clearance = get_radius_and_clearance()

    # Ask for start x, y coordinate
    x_s, y_s = get_valid_start_coordinates(radius, clearance)

    # Ask for start orientation
    theta_s = get_valid_start_orientation()

    # Ask for goal x, y coordinate
    x_g, y_g = get_valid_goal_coordinates(radius, clearance)

    # Ask for goal orientation
    theta_g = get_valid_goal_orientation()

    # Get step size
    global STEP
    STEP = get_valid_step_size()
    
    

    #draw_map(radius, clearance)
    draw_map()

    start = Point(x=x_s, y=y_s, theta=theta_s)
    goal = GoalPt(x=x_g, y=y_g, radius=GOAL_THRESHOLD)

    start_time = time.perf_counter()

    _search = Search(radius, clearance)
    search_success = _search.a_star(start, goal)
    end_time = time.perf_counter()
    time_dict["ASTAR"] = end_time - start_time

    if search_success:
        path = _search.backtrack_path()
        _search.draw_explored()
        draw_path(path)
        draw_path_arrows(path)
    else:
        print("No feasible path found.")

    # Display the map window with OpenCV
    cv2.imshow("Path Visualization", canvas)
    
    
    # # Main function to generate the map
    # canvas, map = initialize_map()
    # update_map(map)
    # canvas[:, :] = map  # Transfer map to canvas
    # display_resized_map(canvas)
    # save_map_image(canvas)

# ===================================== EXECUTE PROGRAM =====================================#

if __name__ == "__main__":
    main()
    for time in time_dict:
        print(time, time_dict[time])

