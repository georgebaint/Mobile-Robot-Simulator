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
from forward_kinematics import ForwardKinematics
from controls import Controls
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
        self.forward_kinematics = ForwardKinematics(self.agent, self.environment)
        self.controls = Controls()
        self.previous_positions = []
        self.previous_estimated_positions = []

    def update(self):
        take_snapshot = self.controls.user_input(self.agent)
        self.forward_kinematics.calculate_forward_kinematics()

        self.agent.kalman_filter(self.forward_kinematics, take_snapshot)
        # self.agent.calculate_forward_kinematics(take_snapshot)
        # self.environment.detect_landmarks(self.forward_kinematics.agent_pos)

        # self.agent.robot_rotation()
        self.agent.rect.center = self.forward_kinematics.agent_pos

        screen.blit(self.agent.image, self.agent.rect)
        self.agent.run_sensors(screen)
        self.environment.draw_landmarks(screen)
        screen.blit(self.agent.image, self.agent.rect)
        pygame.draw.circle(screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
        pygame.draw.rect(screen, "green", self.agent.rect, width=2)

        self.draw_trajectory(screen)  # Change 'solid' to 'dotted' as needed

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

    def draw_trajectory(self, screen):
        # Record the current position of the agent
        self.previous_positions.append((int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)))
        self.previous_estimated_positions.append((int(self.agent.estimated_pos.x), int(self.agent.estimated_pos.y)))

        count = 0

        # Redraw all previously drawn circles
        for pos1, pos2 in zip(self.previous_positions, self.previous_estimated_positions):
            # gray is for solid and the other colour will be used for kalman prediction
            pygame.draw.circle(screen, 'black', pos1, 2)
            # pygame.draw.circle(screen, 'gray', (pos2[0]+20,pos2[1]+20), 2)
            pygame.draw.circle(screen, 'gray', (pos2[0], pos2[1]), 2)

            count = len(self.previous_positions)
            if count > 600:
                self.previous_positions.pop(0)
                self.previous_estimated_positions.pop(0)

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
            # print(pygame.time.get_ticks())

if __name__ == "__main__":
    sim = Simulation()
    sim.run_simulation()
