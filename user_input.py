from obstacle_map import *
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
