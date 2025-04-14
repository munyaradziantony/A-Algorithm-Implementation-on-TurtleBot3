from math import sqrt

LIN_QUANT = 0.5
ANG_QUANT = 1


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
        """Quantize x and y to a 1 mm grid"""
        quantized_x = round(self.x / LIN_QUANT) * LIN_QUANT
        quantized_y = round(self.y / LIN_QUANT) * LIN_QUANT
        # Quantize theta to bins of 30 degrees
        quantized_theta = round(self.theta / ANG_QUANT) * ANG_QUANT
        return hash((quantized_x, quantized_y, quantized_theta))

    def __eq__(self, other):
        """Two nodes are considered equal if they are within 0.5 units and their theta difference is ≤ 30 degrees."""
        if isinstance(other, Point):
            euclidean_distance = sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(
                theta_diff, 360 - theta_diff
            )  # Normalize to range [0, 180]
            return euclidean_distance <= LIN_QUANT and theta_diff <= ANG_QUANT
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
        self.waypoints: list = None
        self.total_cost = 0
        self.parent = None
        self.visited = False

    # used for duplicate key finding in dict
    def __hash__(self):
        """Quantize x and y to a 1 mm grid"""
        quantized_x = round(self.x / LIN_QUANT) * LIN_QUANT
        quantized_y = round(self.y / LIN_QUANT) * LIN_QUANT
        # Quantize theta to bins of 10 degrees
        quantized_theta = round(self.theta / ANG_QUANT) * ANG_QUANT
        return hash((quantized_x, quantized_y, quantized_theta))

    def __eq__(self, other):
        """Two nodes are considered equal if they are within 0.5 units and their theta difference is ≤ 30 degrees."""
        if isinstance(other, Node):
            euclidean_distance = sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)
            theta_diff = abs(self.theta - other.theta)
            theta_diff = min(
                theta_diff, 360 - theta_diff
            )  # Normalize to range [0, 180]
            return euclidean_distance <= LIN_QUANT and theta_diff <= ANG_QUANT
        return False

    def __lt__(self, other):
        if isinstance(other, Node):
            return self.total_cost < other.total_cost
        return False

    def __str__(self):
        """Utility function to print Node directly"""
        return f"Node(x={self.x}, y={self.y}, θ={self.theta}, c2c={self.c2c}, totalcost={self.total_cost})"


# Container to store goal
class GoalPt:
    """
    Goal pt class to hold X Y Radius of goal point
    """

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def __str__(self):
        return f"GoalPt(x={self.x}, y={self.y}, radius={self.radius})"
