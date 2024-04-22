import pygame
import numpy as np 
import cv2 as cv
from settings import *
import scipy

class Environment:
    def __init__(self):       
        self.maze_array = self.get_pixel_map("images/maze.png")  # Get binary pixel map of the maze

    def get_pixel_map(self, image_path, threshold=127):
        # Load the image using OpenCV
        image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
        # Apply a binary threshold to distinguish walls from free space
        _, binary_image = cv.threshold(image, threshold, 255, cv.THRESH_BINARY_INV)
        # Convert binary image to match the display dimensions
        binary_image = cv.resize(binary_image, (WIDTH, HEIGHT))
        return binary_image

    def is_wall(self, x, y):
        # Ensure x and y are within the bounds of the maze array
        if 0 <= x < self.maze_array.shape[1] and 0 <= y < self.maze_array.shape[0]:
            return self.maze_array[y][x] != 0
        else:
            # If out of bounds, treat as a wall to prevent errors
            return True

    def detect_collision(self, old_pos, new_pos, take_snapshot):

        circle = np.zeros((HEIGHT, WIDTH), dtype='uint8')
        circle = cv.circle(circle, (int(new_pos[0]), int(new_pos[1])), ROBOT_RADIUS, 255, -1)

        print(new_pos)

        inter = circle & self.maze_array       
        # print(f"Circle: {circle}")
        # print(f": {circle}") 

        coords = np.argwhere(inter != 0)

        # print(self.maze_array)
        if coords.size == 0:
            return new_pos, False

        #check if the motion is outward
        circle_old = np.zeros((HEIGHT, WIDTH), dtype='uint8')
        circle_old = cv.circle(circle_old, (int(old_pos[0]), int(old_pos[1])), ROBOT_RADIUS, 255, -1)
        inter_old = circle_old & self.maze_array

        inter = inter.astype('bool')
        inter_old = inter_old.astype('bool')

        if (np.sum(inter) <= np.sum(inter_old)):
            return new_pos, False

        #if 2 points of contact => full stop
        footprint = np.zeros((HEIGHT, WIDTH), dtype='int')
        footprint[coords[:, 0], coords[:, 1]] = 1
        labeled, num_labels = scipy.ndimage.label(footprint, structure=np.ones((3, 3)))

        if num_labels > 1:
            #print(set(labeled.flatten())) 
            return old_pos, True

        ys = coords[:, 0]
        xs = coords[:, 1]

        rng = (np.max(xs) - np.min(xs), np.max(ys) - np.min(ys))

        if take_snapshot:
            print(rng)

            plt.imshow(self.maze_array ^ circle, cmap='gray')
            plt.show()

        #return if any collision => full stop
        #return old_pos, True

        # vertical collision
        if rng[0] > rng[1]:
            adj_pos = (new_pos[0], old_pos[1])
            return adj_pos, True

        # horizontal collision
        if rng[1] > rng[0]:
            adj_pos = (old_pos[0], new_pos[1])
            #return adj_pos, True
            return adj_pos, True
        
        return new_pos, True