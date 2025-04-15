# Created by arthavnuc

import numpy as np
import heapq
import time
import cv2
from collections import deque
from canvas import Canvas
from helpers import *
import threading
from robot import Robot
from math import hypot

# A container to hold multiple time values in key: value pair to be used for analysis at the termination of program
time_dict = {}

# Defining some colors for visualization in BGR
#  obstacle space
# WHITE denotes free space
WHITE = (255, 255, 255)
# final path tracing
RED = (0, 0, 255)
# explored nodes
GREEN = (0, 255, 0)
NAVY_BLUE = (42, 27, 13)

BLUE = (255, 0, 0)


file = open("logs.txt", "w")


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

    def __init__(self, robot, canvas):
        # A container to store nodes in dictionary. The dict to store only unique nodes
        self.goal_reached = False
        self.nodes_dict = {}
        self.path: list[Node] = []
        # A container to store the nodes
        self.queue = []
        self.robot: Robot = robot
        self.canvas: Canvas = canvas
        self.search_start = self.get_start()
        self.search_goal = self.get_goal()
        self.search_last_node = None
        self.robot.RPM1, self.robot.RPM2 = self.get_wheel_rpms()
        # self.search_start = Point(500, HEIGHT - 300, -90)
        # self.search_goal = GoalPt(5000, HEIGHT - 3000, 150)

    def get_start(self):
        print("Enter Start Coordinates (x y):")
        while True:
            try:
                x_s, y_s, theta_s = map(
                    int,
                    input(
                        f"Start (x y θ) [Note: (0 ≤ x ≤ {self.canvas.width - 1}), (0 ≤ y ≤ {self.canvas.height - 1}), (-180 ≤ θ < 180)]: "
                    ).split(),
                )

                if not (0 <= x_s < self.canvas.width and 0 <= y_s < self.canvas.height):
                    print("Coordinates are out of bounds... Try again")
                    continue

                if self.canvas.is_colliding(x_s, self.canvas.height / 2 - y_s):
                    print("Start position is colliding... Try again")
                    continue

                if not (-180 <= theta_s < 180):
                    print("Orientation is out of bounds... Try again")

                print(
                    f"Start point validated: Start (x y θ) = ({x_s}, {y_s}, {theta_s})"
                )
                return Point(x_s, self.canvas.height / 2 - y_s, theta_s)

            except ValueError:
                print("Error: Enter three numbers separated by a space")

    def get_goal(self):
        print("Enter Goal Coordinates (x y):")
        while True:
            try:
                x_g, y_g, radius = map(
                    int,
                    input(
                        f"Goal (x y R) [Note: (0 ≤ x ≤ {self.canvas.width - 1}), (0 ≤ y ≤ {self.canvas.height - 1})]: "
                    ).split(),
                )

                if not (0 <= x_g < self.canvas.width and 0 <= y_g < self.canvas.height):
                    print("Coordinates are out of bounds... Try again")
                    continue

                if self.canvas.is_colliding(x_g, self.canvas.height / 2 - y_g):
                    print("Goal position is colliding... Try again")
                    continue

                print(f"Goal point validated: Goal (x y R) = ({x_g}, {y_g}, {radius})")
                return GoalPt(x_g, self.canvas.height / 2 - y_g, radius)

            except ValueError:
                print("Error: Enter three numbers separated by a space")

    def get_wheel_rpms(self):
        while True:
            try:
                RPM1, RPM2 = map(
                    float,
                    input(f"Enter Wheel RPMs (RPM1 RPM2) (must be > 0): ").split(),
                )

                if not (RPM1 > 0 and RPM2 > 0):
                    print("RPM values cannot be non-positive")

                print(f"Wheel RPMs validated: (RPM1 RPM2) = ({RPM1}, {RPM2})")
                return (RPM1, RPM2)

            except ValueError:
                print("Invalid input. Please enter numeric values for RPMs.")

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
        canvas_copy = self.canvas.canvas.copy()
        while not stop_event.is_set():
            new_data = dataq.get_all()
            if new_data:
                for x, y in new_data:
                    cv2.circle(
                        canvas_copy,
                        (int(x), int(y)),
                        5,
                        RED,
                        5,
                    )
                scaled_canvas = cv2.resize(
                    canvas_copy,
                    (
                        int(self.canvas.width * self.canvas.multiplier),
                        int(self.canvas.height * self.canvas.multiplier),
                    ),
                    interpolation=cv2.INTER_AREA,
                )
                cv2.imshow(f"Map Scaled {self.canvas.multiplier} times", scaled_canvas)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def heuristic(self, node: Point, goal: Point):
        """
        Heuristic function for A*. Since C-space is R^2 using Euclidean distance
        Args:
            node: point 1
            goal: point 2

        Returns: Euclidean distance between node and goal

        """
        return hypot(goal.x - node.x, goal.y - node.y)

    def a_star(self):
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

        self.nodes_dict.clear()
        start_node = Node(
            self.search_start.x, self.search_start.y, self.search_start.theta, 0
        )
        start_node.total_cost = self.heuristic(self.search_start, self.search_goal)
        valid_actions = self.robot.valid_actions

        print(f"Start:{start_node}")
        print(f"Goal:{self.search_goal}")
        heapq.heappush(self.queue, start_node)
        self.goal_reached = False
        self.nodes_dict = {
            Point(start_node.x, start_node.y, start_node.theta): start_node
        }

        early_exit = False
        try:
            while self.queue:
                if early_exit:
                    break

                node: Node = heapq.heappop(self.queue)
                if self.reached_goal(node.x, node.y, self.search_goal):
                    self.nodes_dict["last"] = node
                    self.search_last_node = node
                    self.goal_reached = True
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
                        # data_queue.put((wp.x, wp.y))
                        if self.canvas.is_colliding(round(wp.x), round(wp.y)) or not (
                            0 <= wp.x < self.canvas.width
                            and 0 <= wp.y < self.canvas.height
                        ):
                            is_colliding = True
                            break

                        # Not colliding, but check for the cornercase where the waypoint itself is the near the Goal
                        if self.reached_goal(wp.x, wp.y, self.search_goal):
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
                        wp_goal_node.total_cost = wp_goal_node.c2c + self.heuristic(
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
                        self.goal_reached = True
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
                        next_node.total_cost = next_node.c2c + self.heuristic(
                            Point(next_node.x, next_node.y, 0), self.search_goal
                        )
                        next_node.visited = True
                        next_node.parent = node
                        heapq.heappush(self.queue, next_node)
                        self.nodes_dict[
                            Point(next_node.x, next_node.y, next_node.theta)
                        ] = next_node
                        next_node.waypoints = waypoints
                        print(f"Added node: {next_node}")
                        file.write(f"Added node: {next_node}\n")
                    else:
                        print(
                            f"Discarded node: {next_node} because {next_node.visited == False} and {next_node.c2c > node.c2c + cost}"
                        )
                        file.write(
                            f"Discarded node: {next_node} because {next_node.visited == False} and {next_node.c2c > node.c2c + cost}\n"
                        )

        except KeyboardInterrupt:
            pass

        finally:
            # stop_event.set()
            self.plotter_thread.join()
            cv2.destroyAllWindows()
            print("Program terminated.")

        if not self.goal_reached:
            print(f"No path found! Queue: {self.queue}")
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
        end_time = time.perf_counter()
        # time_dict["Backtracking"] = end_time - start_time
        self.path: list[Node] = path

    def animate_search(self):
        """
        Visualizes the explored nodes of graph.
        """
        if not self.goal_reached:
            return

        canvas_copy = self.canvas.canvas.copy()
        cv2.circle(
            canvas_copy,
            (round(self.search_start.x), round(self.search_start.y)),
            15,
            GREEN,
            15,
        )
        cv2.circle(
            canvas_copy,
            (round(self.search_goal.x), round(self.search_goal.y)),
            self.search_goal.radius,
            BLUE,
            5,
        )

        for point, node in self.nodes_dict.items():
            for wp in node.waypoints:
                cv2.circle(canvas_copy, (int(wp.x), int(wp.y)), 5, RED, 5)

            scaled_canvas = cv2.resize(
                canvas_copy,
                (
                    round(self.canvas.width * self.canvas.multiplier),
                    round(self.canvas.height * self.canvas.multiplier),
                ),
                interpolation=cv2.INTER_AREA,
            )
            cv2.imshow(f"Map Scaled {self.canvas.multiplier} times", scaled_canvas)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        for node in self.path:
            for wp in node.waypoints:
                cv2.circle(canvas_copy, (round(wp.x), round(wp.y)), 20, NAVY_BLUE, 20)

            scaled_canvas = cv2.resize(
                canvas_copy,
                (
                    round(self.canvas.width * self.canvas.multiplier),
                    round(self.canvas.height * self.canvas.multiplier),
                ),
                interpolation=cv2.INTER_AREA,
            )
            cv2.imshow(f"Map Scaled {self.canvas.multiplier} times", scaled_canvas)
            time.sleep(0.1)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.waitKey(0)
        cv2.destroyAllWindows()
