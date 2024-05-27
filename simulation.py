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
    """
    Manages the overall simulation environment.

    Attributes:
        environment (Environment): The environment object that contains walls and landmarks.
        agent (Agent): The robot agent that navigates through the environment.
        forward_kinematics (ForwardKinematics): Manages the computation of the robot's position based on the speed of the motors.
        controls (Controls): Handles user input for controlling the robot.
        previous_positions (list): Tracks the historical positions of the robot for plotting reasons.
        previous_estimated_positions (list): Tracks the historical estimated positions from the Kalman filter for plotting reasons.
    """

    def __init__(self):
        """
        Initializes the simulation environment and its components.
        """
        super().__init__()
        self.environment = Environment()
        self.agent = Agent(self.environment)
        self.forward_kinematics = ForwardKinematics(self.agent, self.environment)
        self.controls = Controls()
        self.previous_positions = []
        self.previous_estimated_positions = []


    def draw_text(self, screen, text, position, font_size=24, color='red'):
        """
        Draws text on the screen at the specified position.

        Args:
            screen (Surface): The Pygame surface where text will be drawn.
            text (str): The text to be drawn.
            position (tuple): The (x, y) position for the text on the screen.
            font_size (int): The font size for the text.
            color (str): The color of the text.
        """
        font = pygame.font.Font(None, font_size)  # None uses the default font, set font path for custom font
        text_surface = font.render(text, True, color)  # True means anti-aliased text.
        screen.blit(text_surface, position)

    def get_pixel_map(self, image_path, threshold=127):
        """
        Converts an image to a binary pixel map that indicates walls and passable areas.

        Args:
            image_path (str): The path to the image file.
            threshold (int): The threshold value for binary conversion.

        Returns:
            numpy.ndarray: A binary array representing the passable areas.
        """
        image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
        # Apply a binary threshold to distinguish walls from free space
        _, binary_image = cv.threshold(image, threshold, 255, cv.THRESH_BINARY)
        # Convert binary image to match the display dimensions
        binary_image = cv.resize(binary_image, (WIDTH, HEIGHT))
        return binary_image

    def draw_trajectory(self, screen):
        """
        Draws the trajectory of the robot and its estimated position on the screen.

        Args:
            screen (Surface): The Pygame surface where the trajectory will be drawn.
        """
        self.previous_positions.append((int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)))
        self.previous_estimated_positions.append((int(self.agent.estimated_pos.x), int(self.agent.estimated_pos.y)))

        # Redraw all previously drawn circles
        for pos1, pos2 in zip(self.previous_positions, self.previous_estimated_positions):
            # gray is for solid and the other colour will be used for kalman prediction
            pygame.draw.circle(screen, 'black', pos1, 2)
            pygame.draw.circle(screen, 'gray', (pos2[0], pos2[1]), 2)

            if len(self.previous_positions) > 200:
                self.previous_positions.pop(0)
                self.previous_estimated_positions.pop(0)

    def update(self):
        """
        Updates the state of the simulation for each frame, handling user input, kinematics calculations,
        collision detection, and rendering.
        """
        take_snapshot = self.controls.user_input(self.agent)

        # Optionally we can have noise on the motors
        # self.agent.noise_on_motion_control(0.05, 0.1, 0.07, 0.1)

        self.forward_kinematics.calculate_forward_kinematics()

        self.agent.kalman_filter(self.forward_kinematics, take_snapshot)
        # self.agent.calculate_forward_kinematics(take_snapshot)
        # self.environment.detect_landmarks(self.forward_kinematics.agent_pos)

        self.agent.rect.center = self.forward_kinematics.agent_pos

        screen.blit(self.agent.image, self.agent.rect)
        distances = self.agent.run_sensors(screen)
        self.environment.draw_landmarks(screen)
        screen.blit(self.agent.image, self.agent.rect)
        pygame.draw.circle(screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
        # pygame.draw.rect(screen, "green", self.agent.rect, width=2)

        self.draw_trajectory(screen)

        # Display the current wheel speeds
        self.draw_text(screen, f"Left Wheel Speed: {self.agent.left_motor_speed:.2f}", (10, 10))
        self.draw_text(screen, f"Right Wheel Speed: {self.agent.right_motor_speed:.2f}", (10, 40))


        self.draw_text(screen, f"Real position {self.forward_kinematics.agent_pos[0]:.0f}, {self.forward_kinematics.agent_pos[1]:.0f}", (1000,570))
        self.draw_text(screen, f"Real angle {self.forward_kinematics.agent_angle:.0f}", (1000,590))
        self.draw_text(screen, f"Estimated position {self.agent.estimated_pos[0]:.0f}, {self.agent.estimated_pos[1]:.0f}", (1000,610))
        self.draw_text(screen, f"Estimated angle {self.agent.estimated_angle:.0f}", (1000,630))

    def run_simulation(self):
        """
        Main loop for running the simulation, handling initialization and continuous updating of the simulation state.
        """
        background = pygame.image.load("images/background.jpg").convert()
        background = pygame.transform.scale(background, (WIDTH, HEIGHT))
        maze_image = self.get_pixel_map("images/second_maze.png")  # Get binary pixel map of the maze

        while True:
            keys = pygame.key.get_pressed()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    exit()
            
            screen.blit(background, (0,0))
            maze_surface = pygame.surfarray.make_surface(maze_image.swapaxes(0, 1))
            screen.blit(maze_surface, (0,0))
            
            self.update()

            pygame.display.update()
            clock.tick(FPS)

if __name__ == "__main__":
    sim = Simulation()
    sim.run_simulation()
