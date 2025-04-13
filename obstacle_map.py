import cv2

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
