#! /usr/bin/python3

import heapq
import math

# import threading
# import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import pi as PI
from math import sin
from math import cos
from math import radians

# A container to hold multiple time values in key: value pair to be used for analysis at the termination of program
time_dict = {}

# Set the size of the map
WIDTH, HEIGHT = 5400, 3000

# # Scale factor for resizing

# Colors (BGR format for OpenCV)
RED = (0, 0, 255)
YELLOW = (0, 255, 255)
BLUE = (255, 0, 0)
NEON_GREEN = (20, 255, 57)
NAVY_BLUE = (42, 27, 13)
SILVER_GRAY = (176, 176, 176)
BLACK = (0, 0, 0)

# Robot variables
r = 80  # Robot Radius
R = 25  # Wheel Radius
L = 160  # Wheel Distance
C = 100  # Default
GOAL_THRESHOLD = 1.5

TIME_STEP = 0.1


def get_clearance():
    while True:
        try:
            clearance = int(
                input("Enter the robot clearance (must be an integer > 0): ")
            )
            if clearance > 0:
                return clearance
            else:
                print("Clearance must be greater than 0.")
        except ValueError:
            print("Invalid input. Please enter a valid integer.")


# scale use input and validation
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
    obstacle_upper_left = 2100 <= x <= 2200 and 0 <= y <= 2000
    obstacle_upper_right = 3200 <= x <= 3300 and 0 <= y <= 1000
    obstacle_bottom_left = 1000 <= x <= 1100 and 1000 <= y <= 3000
    obstacle_bottom_centre = 3200 <= x <= 3300 and 2000 <= y <= 3000
    obstacle_bottom_right = 4300 <= x <= 4400 and 1000 <= y <= 3000

    return any(
        [
            obstacle_upper_left,
            obstacle_upper_right,
            obstacle_bottom_left,
            obstacle_bottom_centre,
            obstacle_bottom_right,
        ]
    )


# Function to check if a point is inside a buffer zone
def is_inside_buffer(x, y):
    obstacle_upper_left_BUFFER = (
        2100 - BUFFER <= x <= 2200 + BUFFER and 0 <= y <= 2000 + BUFFER
    )
    obstacle_upper_right_BUFFER = (
        3200 - BUFFER <= x <= 3300 + BUFFER and 0 <= y <= 1000 + BUFFER
    )
    obstacle_bottom_left_BUFFER = (
        1000 - BUFFER <= x <= 1100 + BUFFER and 1000 - BUFFER <= y <= 3000
    )
    obstacle_bottom_centre_BUFFER = (
        3200 - BUFFER <= x <= 3300 + BUFFER and 2000 - BUFFER <= y <= 3000
    )
    obstacle_bottom_right_BUFFER = (
        4300 - BUFFER <= x <= 4400 + BUFFER and 1000 - BUFFER <= y <= 3000
    )

    x_left_boundary = 0 <= x < BUFFER
    x_right_boundary = WIDTH - BUFFER <= x < WIDTH
    y_upper_boundary = 0 <= y < BUFFER
    y_lower_boundary = HEIGHT - BUFFER <= y < HEIGHT

    return any(
        [
            obstacle_upper_left_BUFFER,
            obstacle_upper_right_BUFFER,
            obstacle_bottom_left_BUFFER,
            obstacle_bottom_centre_BUFFER,
            obstacle_bottom_right_BUFFER,
            x_left_boundary,
            x_right_boundary,
            y_upper_boundary,
            y_lower_boundary,
        ]
    )


# Function to update the map based on obstacles and buffer zones
def update_map(map):
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if is_inside_obstacle(x, y):
                map[y, x] = BLACK  # Assign SILVER_GRAY for obstacles
            elif is_inside_buffer(x, y):
                map[y, x] = NEON_GREEN  # Assign NEON_GREEN for clearance


def draw_start_point(canvas, x, y):
    cv2.circle(canvas, (x, y), 10, (0, 0, 255), 10)


def draw_end_point(canvas, x, y):
    cv2.circle(canvas, (x, y), 10, (255, 0, 0), 10)


# Function to resize and display the map
def display_resized_map(map):
    small_map = cv2.resize(
        map, (WIDTH // SCALE, HEIGHT // SCALE), interpolation=cv2.INTER_AREA
    )
    cv2.imshow(f"Map Scaled {SCALE} times smaller", small_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Function to save the map image
def save_map_image(map, SCALE):
    small_map = cv2.resize(
        map, (WIDTH // SCALE, HEIGHT // SCALE), interpolation=cv2.INTER_AREA
    )
    cv2.imwrite("scaled_map_with_BUFFER_output.png", small_map)


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
                print(
                    "Start position is too close to an obstacle (in buffer zone). Try again."
                )
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
            theta_s = float(
                input("Enter start orientation in degrees (-180 ≤ θ < 180): ")
            )
            if -180 <= theta_s < 180:
                return theta_s
            else:
                print(
                    "Orientation must be between -180 (inclusive) and 180 (exclusive)."
                )
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
                print(
                    "Goal position is too close to an obstacle (in buffer zone). Try again."
                )
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
                print("RPM values cannot be non-positive")

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
        return hash((quantized_x, quantized_y))

    def __eq__(self, other):
        """Two nodes are considered equal if they are within 0.5 units and their theta difference is ≤ 30 degrees."""
        if isinstance(other, Point):
            euclidean_distance = math.sqrt(
                (self.x - other.x) ** 2 + (self.y - other.y) ** 2
            )
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(
                theta_diff, 360 - theta_diff
            )  # Normalize to range [0, 180]
            return euclidean_distance <= 0.5 and theta_diff <= 30
        return False

    def __str__(self):
        """utility function to print Point directly"""
        return f"Point({self.x}, {self.y}, {self.theta})"

class WayPoint(Point):
    def __init__(self, x, y, theta, edgecost):
        super().__init__(x, y, theta)
        self.edgecost = edgecost

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
        self.waypoints = None
        self.total_cost = 0
        self.parent = None
        self.visited = False

    # used for duplicate key finding in dict
    def __hash__(self):
        """Quantize x and y to a 0.5 unit grid and multiple of 5 degrees"""
        quantized_x = round(self.x * 2) / 2  # Round to nearest 0.5
        quantized_y = round(self.y * 2) / 2

        # Quantize theta to bins of 1 degrees
        quantized_theta = round(self.theta)

        return hash((quantized_x, quantized_y, quantized_theta))

    def __eq__(self, other):
        """Two nodes are considered equal if they are within 0.5 units and their theta difference is ≤ 30 degrees."""
        if isinstance(other, Node):
            euclidean_distance = math.sqrt(
                (self.x - other.x) ** 2 + (self.y - other.y) ** 2
            )
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(
                theta_diff, 360 - theta_diff
            )  # Normalize to range [0, 180]
            return euclidean_distance <= 0.5 and theta_diff <= 30
        return False

    def __lt__(self, other):
        if isinstance(other, Node):
            return self.total_cost < other.total_cost
        return False

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


# ==========================================================================================
# Cost to go heuristic using Euclidean distance
def heuristic_cost(x, y):
    goal_x, goal_y = goal_node
    dist = math.sqrt(((goal_x - x) ** 2) + ((goal_y - y) ** 2))
    return dist


# Compute the edge cost based on translational and rotational motion due to the action
def edge_cost(u_l, u_r):
    # Translational component of velocity
    linear_vel = (r / 2) * (u_l + u_r)

    # Rotational component
    angular_vel = (r / L) * (u_r - u_l)

    # Cost = Translational + Rotational
    c2c = (linear_vel + abs(angular_vel)) * TIME_STEP
    return c2c


# Function to get the angular velocity from RPM
def rpm_to_rad_ps(RPM):
    return (2 * PI * RPM) / 60


def get_waypoints_cost(node: Node, u_l, u_r):
    edgecost = 0
    x_i = node.x
    y_i = node.y
    theta_i = radians(node.theta)
    waypoints = []
    for _ in np.arange(0, 1, 0.1):
        dx = 0.5 * R * (u_l + u_r) * cos(theta_i) * TIME_STEP
        dy = 0.5 * R * (u_l + u_r) * sin(theta_i) * TIME_STEP
        dtheta = (R / L) * (u_r - u_l) * TIME_STEP
        dcost = (R / 2) * (u_l + u_r) * TIME_STEP
        edgecost += dcost
        x_i += dx
        y_i += dy
        theta_i += dtheta
        waypoints.append(WayPoint(x_i, y_i, theta_i, edgecost))
    return waypoints, edgecost


# Defining set of actions based on wheel RPMs
def action(node, RPM_RW, RPM_LW):
    # Convert the units from RPM to radians/second
    u_l = rpm_to_rad_ps(RPM_LW)
    u_r = rpm_to_rad_ps(RPM_RW)
    return get_waypoints_cost(node, u_l, u_r)


def sharp_right_R1(node):
    return action(
        node, RPM1, 0
    )  # Pivoting right by applying RPM1 on the left wheel and 0 on the right wheel


def sharp_left_R1(node):
    return action(
        node, 0, RPM1
    )  # Pivoting left by applying 0 on the left wheel and 0 on the right wheel


def straight_R1(node):
    return action(node, RPM1, RPM1)  # Move straight by applying RPM1 on both wheels


def sharp_right_R2(node):
    return action(
        node, RPM2, 0
    )  # Pivoting right by applying RPM2 on the left wheel and 0 on the right wheel


def sharp_left_R2(node):
    return action(
        node, 0, RPM2
    )  # Pivoting left by applying 0 on the left wheel and RPM2 on the right wheel


def straight_R2(node):
    return action(node, RPM2, RPM2)  # Move straight by applying RPM2 on both wheels


def gradual_turn_R1R2(node):
    return action(
        node, RPM1, RPM2
    )  # Turn by applying RPM1 on right wheel and RPM2 on left wheel


def gradual_turn_R2R1(node):
    return action(
        node, RPM2, RPM1
    )  # Turn by applying RPM2 on right wheel and RPM1 on left wheel


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

    def __init__(self, radius, clearance):
        # A container to store nodes in dictionary. The dict to store only unique nodes
        self.nodes_dict = {}
        # A container to store the nodes
        self.queue = []
        self.search_goal = None
        self.search_start = None
        self.search_last_node = None
        self.rad = radius
        self.clear = clearance

    def reached_goal(self, x, y, goal: GoalPt):
        """
        Termination condition if robot reached the goal region, In this Project ignoring the goal theta  so basically robot can be in any orientation in goal region
        :param x: Current X coordinate
        :param y: Current Y coordinate
        :param goal: Goal region (X,Y,Radius)
        :return: Boolean
        """
        if (x - goal.x) ** 2 + (y - goal.y) ** 2 < goal.radius**2:
            return True
        else:
            return False

    def a_star(self, start: Point, goal: GoalPt):
        """
        Main function for A* algorithm. All the valid actions are defined here. Robot is considered in a Hex connected grid, and it can go from -180 deg to 180 deg.
        There are 12 valid actions defined having format dx dy dtheta cost. cost is basically distance to travel and dx dy dtheta is change is position and orientation.
        The valid actions is scaled as per STEP size input from the user.
        Args:
            start: start point(X,Y,Theta) in graph
            goal: goal region (X,Y,Radius) in graph

        Returns: Boolean: True is an optimal path is found, False otherwise

        """
        # start_time = time.perf_counter()
        self.search_start = start
        self.search_goal = Point(goal.x, goal.y, 0)
        self.nodes_dict.clear()
        # Action list for 12 grid
        # each tuple is x y theta cost
        # A robot can take [-180,180]
        R3 = 3**0.5
        # valid_actions = {(STEP, 0, 0, STEP), (1.5 * STEP, (R3 * STEP) / 2, 30, R3 * STEP),
        #                  (0.5 * STEP, (R3 * STEP) / 2, 60, STEP), (0, R3 * STEP, 90, R3 * STEP),
        #                  (-0.5 * STEP, (R3 * STEP) / 2, 120, STEP), (-1.5 * STEP, (R3 * STEP) / 2, 150, R3 * STEP),
        #                  (-STEP, 0, 180, STEP), (1.5 * STEP, -(R3 * STEP) / 2, -30, R3 * STEP),
        #                  (0.5 * STEP, -(R3 * STEP) / 2, -60, STEP), (0, -R3 * STEP, -90, R3 * STEP),
        #                  (-0.5 * STEP, -(R3 * STEP) / 2, -120, STEP), (-1.5 * STEP, -(R3 * STEP) / 2, -150, R3 * STEP)}

        start_node = Node(start.x, start.y, start.theta, 0)
        start_node.total_cost = heuristic_cost(start_node.x, start_node.y)
        valid_actions = [
            sharp_right_R1,
            sharp_left_R1,
            straight_R1,
            sharp_right_R2,
            sharp_left_R2,
            straight_R2,
            gradual_turn_R1R2,
            gradual_turn_R2R1,
        ]

        u = Node(start.x, start.y, start.theta, 0)
        heapq.heappush(self.queue, u)
        goal_reached = False
        self.nodes_dict = {Point(u.x, u.y, u.theta): u}

        while self.queue:
            node: Node = heapq.heappop(self.queue)
            if self.reached_goal(node.x, node.y, goal):
                self.nodes_dict["last"] = node
                self.search_last_node = node
                goal_reached = True
                print("Path found!")
                break

            for action in valid_actions:
                waypoints, cost = action(node)

                # Check waypoints and endpoint for collisions with the obstacles or the wall
                collisions = False
                goalreached = False
                goalwp:WayPoint = None
                for wp in waypoints:
                    if (
                        is_inside_buffer(wp.x, wp.y)
                        or is_inside_obstacle(wp.x, wp.y)
                        or not (0 <= wp.x < WIDTH and 0 <= wp.y < HEIGHT)
                    ):
                        collisions = True
                        break

                    # Not colliding, but check for the cornercase where the waypoint itself is the near the Goal 
                    if self.reached_goal(wp.x, wp.y, goal):
                        goalreached = True
                        goalwp = wp
                        break

                if collisions:      # One or more waypoints are colliding, so exclude this action and proceed with the next one
                    continue

                if goalreached:
                    self.nodes_dict["last"] = Node(goalwp.x, goalwp.y, goalwp.theta, 0)
                    self.search_last_node = node
                    goal_reached = True
                    print("Path found!")

                # Check in dictionary if the node exists at the point, else create a new node at the last waypoin
                next_node = self.nodes_dict.get(
                    Point(
                        node.x + waypoints[-1].x,
                        node.y + waypoints[-1].y,
                        node.theta + waypoints[-1].theta,
                    ),
                    Node(
                        waypoints[-1].x,
                        waypoints[-1].y,
                        waypoints[-1].theta,
                        0,
                    ),
                )

                if (next_node.c2c > node.c2c + cost) or not next_node.visited:
                    next_node.c2c = node.c2c + cost
                    # Updating total cost to ctoc with heuristic, total cost and parent and marking it as visited
                    next_node.total_cost = next_node.c2c + heuristic(
                        Point(next_node.x, next_node.y, 0), self.search_goal
                    )
                    next_node.visited = True
                    next_node.parent = node
                    heapq.heappush(self.queue, next_node)
                    self.nodes_dict[
                        Point(next_node.x, next_node.y, next_node.theta)
                    ] = next_node
                    next_node.waypoints = waypoints

        if not goal_reached:
            print("No path found!")
            return False
        # end_time = time.perf_counter()
        # time_dict["ASTAR"] = end_time - start_time
        return True

    def backtrack_path(self):
        """
        Backtracks from the goal to the start using the dict
        """
        # start_time = time.perf_counter()

        path = []
        # Backtracking using the parent in node
        g = self.nodes_dict.pop("last")
        for _ in iter(int, 1):
            path.append(g)
            if g.parent.x == self.search_start.x and g.parent.y == self.search_start.y:
                break
            g = g.parent
        path.append(g)
        path.reverse()
        # end_time = time.perf_counter()
        # time_dict["Backtracking"] = end_time - start_time
        return path  # Return the reconstructed shortest path

    # def draw_explored(self):
    #     """
    #     Visualizes the explored nodes of graph.
    #     """
    #     pygame.draw.circle(screen, RED, (self.search_goal.x, self.search_goal.y), 15, 5)
    #     pygame.draw.circle(screen, BLUE, (self.search_start.x, self.search_start.y), 5, 5)
    #     for node in self.nodes_dict:
    #         pygame.draw.circle(screen, RED, (node.x, node.y), 1)

    #     pygame.display.update()


# ========================================================================================================
def visualize_path(map, visited_nodes, optimal_path):
    print("Animating path...")

    display_map = map.copy()

    # Create a window
    cv2.namedWindow("A* Exploration and Path", cv2.WINDOW_NORMAL)
    resized_dims = (WIDTH // SCALE, HEIGHT // SCALE)

    # Animate the explored nodes
    for path, _ in visited_nodes:
        for i in range(len(path) - 1):
            pt1 = (int(path[i][0]), int(path[i][1]))
            pt2 = (int(path[i + 1][0]), int(path[i + 1][1]))
            cv2.line(display_map, pt1, pt2, (0, 255, 255), 1)

        # Resize and show each frame
        frame = cv2.resize(display_map, resized_dims)
        cv2.imshow("A* Exploration and Path", frame)
        cv2.waitKey(1)  # You can increase for slower effect, e.g., 10

    # Animate the optimal path
    for i in range(len(optimal_path) - 1):
        pt1 = (int(optimal_path[i][0]), int(optimal_path[i][1]))
        pt2 = (int(optimal_path[i + 1][0]), int(optimal_path[i + 1][1]))
        cv2.line(display_map, pt1, pt2, (255, 0, 255), 3)

        frame = cv2.resize(display_map, resized_dims)
        cv2.imshow("A* Exploration and Path", frame)
        cv2.waitKey(50)  # Slower to highlight final path

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Save the final image
    save_map_image(display_map, SCALE)


def plot_path_vectors(optimal_path):
    print("Plotting optimal path with vectors...")

    X = []
    Y = []
    U = []
    V = []

    for i in range(len(optimal_path) - 1):
        x0, y0, _ = optimal_path[i]
        x1, y1, _ = optimal_path[i + 1]

        dx = x1 - x0
        dy = y1 - y0

        X.append(x0)
        Y.append(y0)
        U.append(dx)
        V.append(dy)

    X = np.array(X)
    Y = np.array(Y)
    U = np.array(U)
    V = np.array(V)

    fig, ax = plt.subplots()
    ax.quiver(
        X,
        Y,
        U,
        V,
        angles="xy",
        scale_units="xy",
        scale=1,
        color="r",
        headwidth=3,
        headlength=5,
    )

    ax.set_title("Plot of path vectors")
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.grid(True)
    ax.set_aspect("equal")
    plt.gca().invert_yaxis()  # If using OpenCV coordinates (top-left origin)
    plt.show()
    plt.close()


# Main function to run the program
def main():
    global SCALE, C, BUFFER, RPM1, RPM2, goal_node

    SCALE = get_scale_factor()
    C = get_clearance()
    BUFFER = 2 * r + C

    print("Creating the map...")
    map = initialize_map()
    update_map(map)

    startpos = get_start_position()
    start_orientation = get_valid_start_orientation()
    goalpos = get_goal_position()
    RPM1, RPM2 = get_wheel_rpms()
    goal_node = goalpos

    draw_start_point(map, *startpos)
    draw_end_point(map, *goalpos)

    startpt = Point(startpos[0], startpos[1], start_orientation)
    goalpt = GoalPt(goalpos[0], goalpos[1], GOAL_THRESHOLD)

    _search = Search(r, C)
    search_success = _search.a_star(startpt, goalpt)

    if search_success:
        path = _search.backtrack_path()
        print("Path found")
        # _search.draw_explored()
        # draw_path(path)
        # draw_path_arrows(path)
    else:
        print("No feasible path found.")


if __name__ == "__main__":
    main()
