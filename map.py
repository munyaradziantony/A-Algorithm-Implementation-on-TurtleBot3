#! /usr/bin/python3

import heapq
import math

import threading
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from math import pi as PI
from math import radians, degrees, sqrt
from dataclasses import *
from collections import deque
from user_input import *
from obstacle_map import *
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
GOAL_THRESHOLD = 50

TIME_STEP = 0.1

file = open("nodes.txt", "w")




# Function to initialize the map and map
def initialize_map():
    map = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    return map


# ==========================================================================================
# Function to get the angular velocity from RPM
def rpm_to_rad_ps(RPM):
    return (2 * PI * RPM) / 60


# Obtain the neighboring nodes by computing the edge cost
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
        # dcost = (R / 2) * (u_l + u_r) * TIME_STEP
        dcost = math.sqrt(dx**2 + dy**2)
        edgecost += dcost
        x_i += dx
        y_i += dy
        theta_i = radians(principal_theta(degrees(theta_i) + degrees(dtheta)))
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


class DataQueue:
    def __init__(self, max_size=100):
        self.queue = deque()
        self.max_size = max_size
        self.lock = threading.Lock()

    def put(self, item):
        with self.lock:
            if len(self.queue) == self.max_size:
                self.queue.popleft()
            self.queue.append(item)

    def get(self):
        with self.lock:
            if self.queue:
                return self.queue.popleft()
            return None

    def get_all(self):
        with self.lock:
            items = list(self.queue)
            self.queue.clear()
            return items


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

    def plotter(self, dataq: DataQueue, stop_event: threading.Event):
        plt.ion()
        fig, ax = plt.subplots()
        x_data, y_data = [], []
        (line,) = ax.plot(x_data, y_data, "bo-")  # blue circles with lines

        while not stop_event.is_set():
            new_data = dataq.get_all()
            if new_data:
                x_data.extend([point[0] for point in new_data])
                y_data.extend([point[1] for point in new_data])
                line.set_xdata(x_data)
                line.set_ydata(y_data)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()
            else:
                time.sleep(0.1)

        plt.ioff()
        plt.close(fig)

    def a_star(self, start: Point, goal: GoalPt):
        """
        Main function for A* algorithm. All the valid actions are defined here. The robot is non-holonomic and constitutes of 8 actions given as an differential RPM pair:
        This includes (RPM1, 0), (0, RPM1), (RPM2, 0), (0, RPM2), (RPM1, RPM2), (RPM2, RPM1), (RPM1, RPM1), (RPM2, RPM2)
        Args:
            start: start point(X,Y,Theta) in graph
            goal: goal region (X,Y,Radius) in graph
        Returns: Boolean: True is an optimal path is found, False otherwise
        """
        # start_time = time.perf_counter()

        # Create grid of x, y coordinates
        stop_event = threading.Event()
        data_queue = DataQueue()
        self.plotter_thread = threading.Thread(
            target=self.plotter,
            args=(data_queue, stop_event),
        )
        self.plotter_thread.start()

        self.search_start = start
        self.search_goal = Point(goal.x, goal.y, 0)
        self.nodes_dict.clear()
        start_node = Node(start.x, start.y, start.theta, 0)
        start_node.total_cost = heuristic(start, goal)
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
        print(f"Start:{start_node}")
        print(f"Goal:{goal}")

        u = Node(start.x, start.y, start.theta, 0)
        heapq.heappush(self.queue, u)
        goal_reached = False
        self.nodes_dict = {Point(u.x, u.y, u.theta): u}

        early_exit = False
        try:
            while self.queue:
                if early_exit:
                    break

                node: Node = heapq.heappop(self.queue)
                if self.reached_goal(node.x, node.y, goal):
                    self.nodes_dict["last"] = node
                    self.search_last_node = node
                    goal_reached = True
                    print(f"Goal found at {node}")
                    break

                print(f"Exploring node: {node}")
                file.write(f"Exploring node: {node}\n")

                for act_idx, action in enumerate(valid_actions):
                    waypoints, cost = action(node)

                    # Check waypoints and endpoint for collisions with the obstacles or the wall
                    is_colliding = False
                    is_wp_near_goal = False
                    wp_goal_idx: WayPoint = None
                    for wp_idx, wp in enumerate(waypoints):
                        data_queue.put((wp.x, wp.y))
                        if (
                            is_inside_buffer(wp.x, wp.y)
                            or is_inside_obstacle(wp.x, wp.y)
                            or not (0 <= wp.x < WIDTH and 0 <= wp.y < HEIGHT)
                        ):
                            is_colliding = True
                            break

                        # Not colliding, but check for the cornercase where the waypoint itself is the near the Goal
                        if self.reached_goal(wp.x, wp.y, goal):
                            is_wp_near_goal = True
                            wp_goal_idx = wp_idx
                            break

                    if is_colliding:
                        print(
                            f"Collision detected while exploring {node}, discarding action {act_idx}"
                        )
                        file.write(
                            f"Collision detected while exploring {node}, discarding action {act_idx}\n"
                        )
                        continue  # At least one waypoint is colliding, so exclude this action and proceed with the next one

                    if is_wp_near_goal:
                        # The waypoint is near the goal, so add it as a node and proceed
                        wp_goal: WayPoint = waypoints[wp_goal_idx]
                        wp_goal_node: Node = Node(
                            wp_goal.x,
                            wp_goal.y,
                            wp_goal.theta,
                            node.c2c + wp_goal.edgecost,
                        )
                        wp_goal_node.total_cost = wp_goal_node.c2c + heuristic(
                            Point(wp_goal_node.x, wp_goal_node.y, wp_goal_node.theta),
                            self.search_goal,
                        )
                        wp_goal_node.visited = True
                        wp_goal_node.parent = node
                        wp_goal_node.waypoints = waypoints[:wp_goal_idx]
                        self.nodes_dict["last"] = wp_goal_node
                        self.nodes_dict[
                            Point(wp_goal_node.x, wp_goal_node.y, wp_goal_node.theta)
                        ] = wp_goal_node
                        self.search_last_node = wp_goal_node
                        print(f"Goal found near waypoint at {wp_goal_node}")
                        file.write(f"Goal found near waypoint at {wp_goal_node}\n")
                        early_exit = True
                        goal_reached = True
                        break

                    # Check in dictionary if the node exists at the point, else create a new node at the last waypoin
                    next_node = self.nodes_dict.get(
                        Point(
                            waypoints[-1].x,
                            waypoints[-1].y,
                            waypoints[-1].theta,
                        ),
                        Node(
                            waypoints[-1].x,
                            waypoints[-1].y,
                            waypoints[-1].theta,
                            0,
                        ),
                    )
                    if (next_node.c2c > node.c2c + cost) or not next_node.visited:
                        # Updating total cost to ctoc with heuristic, total cost and parent and marking it as visited
                        next_node.c2c = node.c2c + cost
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
                    print(f"New node: {next_node}")
                    file.write(f"New node: {next_node}\n")

        except KeyboardInterrupt:
            pass

        finally:
            stop_event.set()
            self.plotter_thread.join()
            print("Program terminated.")

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
    BUFFER = r + C

    print("Creating the map...")
    map = initialize_map()
    # update_map(map)

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
