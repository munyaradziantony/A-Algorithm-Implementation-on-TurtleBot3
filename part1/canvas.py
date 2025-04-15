import numpy as np
import cv2
BLACK = (0, 0, 0)
GREY = (128, 128, 128)

# class to create canvas having obstacle and boundaries
class Canvas:
    def __init__(self, width, height, buffer=2, multiplier=1):
        self.multiplier = multiplier
        self.width = width
        self.height = height
        self.buffer = buffer
        # using 3D array for color visualization in opencv mat
        #  WHITE canvas
        self.canvas = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255

        print("Preparing Canvas")
        self._draw_borders()
        self._draw_obstacles()

    # Function to draw borders on canvas
    def _draw_borders(self):
        cv2.rectangle(
            img=self.canvas,
            pt1=(0, 0),
            pt2=(self.width, self.height),
            color=BLACK,
            thickness=self.buffer,
        )

    # Function to visualise the canvas for debugging
    def _visualize_canvas(self):
        resized = cv2.resize(
            self.canvas,
            (int(self.width * self.multiplier), int(self.height * self.multiplier)),
            interpolation=cv2.INTER_AREA,
        )
        cv2.imshow("img", resized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Function calling each type of obstacle to be drawn
    def _draw_obstacles(self):
        for x in range(self.width):
            for y in range(self.height):
                if self.is_inside_obstacle(x, y):
                    self.canvas[y, x] = BLACK
                elif self.is_inside_buffer(x, y):
                    self.canvas[y, x] = GREY

    def is_inside_obstacle(self, x, y):
        pillar1 = (1000 <= x <= 1100) and (500 <= y <= 3000)
        pillar2 = (2100 <= x <= 2200) and (0 <= y <= 2500)
        pillar3 = (3200 <= x <= 3300) and (0 <= y <= 1250)
        pillar4 = (3200 <= x <= 3300) and (1750 <= y <= 3000)
        pillar5 = (4300 <= x <= 4400) and (500 <= y <= 3000)
        return any(
            [
                pillar1,
                pillar2,
                pillar3,
                pillar4,
                pillar5,
            ]
        )

    def is_inside_buffer(self, x, y):
        pillar1 = (1000 - self.buffer <= x <= 1100 + self.buffer) and (
            500 - self.buffer <= y <= 3000
        )
        pillar2 = (2100 - self.buffer <= x <= 2200 + self.buffer) and (
            0 <= y <= 2500 + self.buffer
        )
        pillar3 = (3200 - self.buffer <= x <= 3300 + self.buffer) and (
            0 <= y <= 1250 + self.buffer
        )
        pillar4 = (3200 - self.buffer <= x <= 3300 + self.buffer) and (
            1750 - self.buffer <= y <= 3000
        )
        pillar5 = (4300 - self.buffer <= x <= 4400 + self.buffer) and (
            500 - self.buffer <= y <= 3000
        )
        return any(
            [
                pillar1,
                pillar2,
                pillar3,
                pillar4,
                pillar5,
            ]
        )

    def is_colliding(self, x, y):
        return (
            self.canvas[y, x][0] == BLACK[0]
            and self.canvas[y, x][1] == BLACK[1]
            and self.canvas[y, x][2] == BLACK[2]
        ) or (
            self.canvas[y, x][0] == GREY[0]
            and self.canvas[y, x][1] == GREY[1]
            and self.canvas[y, x][2] == GREY[2]
        )
