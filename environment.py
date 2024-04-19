import pygame
import numpy as np 
import cv2 as cv
from settings import *

class Environment:
    def __init__(self):                
        self.maze_array = self.get_pixel_map("images/maze.png")  # Get binary pixel map of the maze

    def get_pixel_map(self, image_path, threshold=127):
        # Load the image using OpenCV
        image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
        # Apply a binary threshold to distinguish walls from free space
        _, binary_image = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)
        # Convert binary image to match the display dimensions
        binary_image = cv.resize(binary_image, (WIDTH, HEIGHT))
        return binary_image

    def is_wall(self, x, y):
        # Ensure x and y are within the bounds of the maze array
        if 0 <= x < self.maze_array.shape[1] and 0 <= y < self.maze_array.shape[0]:
            return self.maze_array[y][x] == 0
        else:
            # If out of bounds, treat as a wall to prevent errors
            return True