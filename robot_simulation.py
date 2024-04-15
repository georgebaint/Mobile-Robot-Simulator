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

    def robot_rotation(self):
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

    def add_sensors(self, screen):
        num_sensors = 12
        sensor_length = 30  # Adjust based on desired sensor reach
        tab20 = plt.get_cmap("tab20").colors
        for i in range(num_sensors):
            angle = math.radians((360 / num_sensors) * i + self.angle)
            end_pos = (
                self.pos.x + (self.radius + sensor_length) * math.cos(angle),
                self.pos.y + (self.radius + sensor_length) * math.sin(angle)
            )
            start_pos = (
                self.pos.x + self.radius * math.cos(angle),
                self.pos.y + self.radius * math.sin(angle)
            )

            # Convert color from float (0.0-1.0) to int (0-255)
            color = tuple(int(255 * x) for x in tab20[i % len(tab20)])
            pygame.draw.line(screen, color, start_pos, end_pos, 2)        


if __name__ == "__main__":
    pygame.init()

    # Creating the window
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption('Mobile Robot Simulation')
    clock = pygame.time.Clock()

    imgray = cv.cvtColor(cv.imread('images/maze.png'), cv.COLOR_BGR2GRAY)
    _, t_maze = cv.threshold(imgray, 127, 255, cv.THRESH_BINARY)

    t_maze_surface = pygame.surfarray.make_surface(t_maze.swapaxes(0, 1))

    # Load images
    background = pygame.transform.scale(pygame.image.load("images/background.jpg").convert(), (WIDTH, HEIGHT))
    maze_mask = pygame.transform.scale(t_maze_surface.convert_alpha(), (WIDTH, HEIGHT))




    robot = Robot()

    while True:
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()

        
        screen.blit(background, (0,0))
        background.blit(maze_mask, (0,0))


        screen.blit(robot.image, robot.rect)
        robot.update()
        robot.add_sensors(screen)

        pygame.draw.circle(screen, "blue", (int(robot.pos.x), int(robot.pos.y)), robot.radius, width=2)
        pygame.draw.rect(screen, "green", robot.rect, width=2)


        pygame.display.update()
        clock.tick(FPS)