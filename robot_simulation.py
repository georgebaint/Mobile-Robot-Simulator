import pygame
from sys import exit
import math
from settings import *
import random
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt



class Robot(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.transform.rotozoom(pygame.image.load("images/robot.png").convert_alpha(), 0, ROBOT_SIZE)
        self.base_robot_image = self.image
        self.pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
        self.speed = ROBOT_SPEED
        # Define the radius of the circular hitbox (eg half the width of the image)
        self.radius = self.base_robot_image.get_width() // 2
        self.rect = self.base_robot_image.get_rect(center=self.pos)
        self.angle = 0

    def robot_rotation(self):
        if self.velocity_x != 0 or self.velocity_y != 0:
            self.angle = math.degrees(math.atan2(self.velocity_x, -self.velocity_y))
        self.image = pygame.transform.rotate(self.base_robot_image, -self.angle)
        self.rect = self.image.get_rect(center=self.pos)

    def user_input(self):
        # TODO Velocity of different wheels
        self.velocity_x = 0
        self.velocity_y = 0

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            self.velocity_y = -self.speed

        if keys[pygame.K_s]:
            self.velocity_y = self.speed

        if keys[pygame.K_a]:
            self.velocity_x = -self.speed

        if keys[pygame.K_d]:
            self.velocity_x = self.speed

        if self.velocity_x != 0 and self.velocity_y != 0:  # Robot is moving diagonally
            self.velocity_x /= math.sqrt(2)  # From Pythagorean theorem
            self.velocity_y /= math.sqrt(2)

    def move(self):
        self.pos += pygame.math.Vector2(self.velocity_x, self.velocity_y)
        self.rect.center = self.pos

    def update(self):
        self.user_input()
        self.move()
        self.robot_rotation()

    def is_wall(self, x, y, maze_array):
        """
        Checks if the given x, y position is a wall in the maze.

        Args:
        x (int): The x-coordinate in the maze.
        y (int): The y-coordinate in the maze.
        maze_array (numpy.ndarray): The array representing the maze, where walls are 0 (black).

        Returns:
        bool: True if the position is a wall, False otherwise.
        """
        # Ensure x and y are within the bounds of the maze array
        if 0 <= x < maze_array.shape[1] and 0 <= y < maze_array.shape[0]:
            return maze_array[y][x] == 0
        else:
            # If out of bounds, treat as a wall to prevent errors
            return True


    def add_sensors(self, screen, maze_array):
        num_sensors = 12
        max_sensor_length = 200
        tab20 = plt.get_cmap("tab20").colors

        for i in range(num_sensors):
            angle = math.radians((360 / num_sensors) * i + self.angle)
            sensor_hit_wall = False
            for distance in range(self.radius, self.radius + max_sensor_length):
                end_x = int(self.pos.x + distance * math.cos(angle))
                end_y = int(self.pos.y + distance * math.sin(angle))

                if self.is_wall(end_x, end_y, maze_array):
                    sensor_hit_wall = True
                    break
            
            if not sensor_hit_wall:
                # If no wall is hit, the sensor extends to its maximum length
                distance = self.radius + max_sensor_length
                

            # Sensor line start and end positions
            start_pos = (
                int(self.pos.x + self.radius * math.cos(angle)),
                int(self.pos.y + self.radius * math.sin(angle))
            )
            end_pos = (
                int(self.pos.x + distance * math.cos(angle)),
                int(self.pos.y + distance * math.sin(angle))
            )

            color = tuple(int(255 * x) for x in tab20[i % len(tab20)])
            pygame.draw.line(screen, color, start_pos, end_pos, 2)      

def get_pixel_map(image_path, threshold=127):
    # Load the image using OpenCV
    image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
    # Apply a binary threshold to distinguish walls from free space
    _, binary_image = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)
    # Convert binary image to match the display dimensions
    binary_image = cv.resize(binary_image, (WIDTH, HEIGHT))
    return binary_image

if __name__ == "__main__":
    pygame.init()

    # Creating the window
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption('Mobile Robot Simulation')
    clock = pygame.time.Clock()

    # Load images
    background = pygame.image.load("images/background.jpg").convert()
    background = pygame.transform.scale(background, (WIDTH, HEIGHT))
    maze_image = get_pixel_map("images/maze.png")  # Get binary pixel map of the maze


    robot = Robot()

    while True:
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        
        screen.blit(background, (0,0))

        # Optionally display the maze for debugging (convert numpy array to surface)
        maze_surface = pygame.surfarray.make_surface(maze_image.swapaxes(0, 1))
        screen.blit(maze_surface, (0,0))
        
        robot.update()

        screen.blit(robot.image, robot.rect)
        robot.add_sensors(screen, maze_image)
        screen.blit(robot.image, robot.rect)
        pygame.draw.circle(screen, "blue", (int(robot.pos.x), int(robot.pos.y)), robot.radius, width=2)
        pygame.draw.rect(screen, "green", robot.rect, width=2)


        pygame.display.update()
        clock.tick(FPS)