import pygame
from settings import *

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
# Creating the window
pygame.display.set_caption('Mobile Robot Simulation')
clock = pygame.time.Clock()

from sys import exit
import math
from agent import Agent
from environment import Environment
from controls import user_input
from settings import *
import random
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

class Simulation(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.environment = Environment()
        self.agent = Agent(self.environment)

    def update(self):
        take_snapshot = user_input(self.agent)
        
        self.agent.calculate_forward_kinematics(take_snapshot)

        # self.agent.robot_rotation()
        self.agent.rect.center = self.agent.pos

        screen.blit(self.agent.image, self.agent.rect)
        self.agent.run_sensors(screen)
        screen.blit(self.agent.image, self.agent.rect)
        pygame.draw.circle(screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.agent.pos.x), int(self.agent.pos.y)), self.agent.radius, width=2)
        pygame.draw.rect(screen, "green", self.agent.rect, width=2)

        # Display the current wheel speeds
        self.draw_text(screen, f"Left Wheel Speed: {self.agent.left_motor_speed:.2f}", (10, 10))
        self.draw_text(screen, f"Right Wheel Speed: {self.agent.right_motor_speed:.2f}", (10, 40))

    def draw_text(self, screen, text, position, font_size=24, color='red'):
        # A function that draws the speed of each motor
        font = pygame.font.Font(None, font_size)  # None uses the default font, set font path for custom font
        text_surface = font.render(text, True, color)  # True means anti-aliased text.
        screen.blit(text_surface, position)

    def get_pixel_map(self, image_path, threshold=127):
        # Load the image using OpenCV
        image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
        # Apply a binary threshold to distinguish walls from free space
        _, binary_image = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)
        # Convert binary image to match the display dimensions
        binary_image = cv.resize(binary_image, (WIDTH, HEIGHT))
        return binary_image

    def run_simulation(self):
        
        background = pygame.image.load("images/background.jpg").convert()
        background = pygame.transform.scale(background, (WIDTH, HEIGHT))
        maze_image = self.get_pixel_map("images/maze.png")  # Get binary pixel map of the maze

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
            
            self.update()

            pygame.display.update()
            clock.tick(FPS)

if __name__ == "__main__":
    sim = Simulation()
    sim.run_simulation()
