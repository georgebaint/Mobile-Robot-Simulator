import numpy as np
import math
import pygame
from settings import *

import pygame
import math

class ForwardKinematics:
    """
    This class handles the calculation of forward kinematics for our mobile robot within the simulated environment.
    determining the new position and orientation of the robot based on its motor commands and previous state.
    
    Attributes:
        agent (Agent): The robot being controlled.
        agent_pos (pygame.math.Vector2): The current position of the robot.
        agent_angle (float): The current orientation angle of the robot in degrees.
        environment (Environment): The simulated environment in which the robot operates.
        landmarks (list): List of landmarks in the environment.
    """
    
    def __init__(self, agent, environment):
        """
        Initializes the ForwardKinematics instance with the given agent and environment.
        """

        self.agent = agent
        self.agent_pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)  # Starting position of the robot.
        self.agent_angle = 0  # Starting orientation of the robot.
        self.environment = environment
        self.landmarks = environment.landmarks

    def calculate_forward_kinematics(self):
        """
        Computes the new position and orientation of the robot based on its wheel speeds.
        This method handles both straight-line and rotational movement. It updates the 
        robot's position, handles collision detection, and updates sensor data accordingly.
        """
        old_pos = (self.agent_pos.x, self.agent_pos.y)  # Preserve old position for collision detection later.
        dt = 1  # Time step in frames.

        # Calculate angular velocity.
        omega = (self.agent.left_motor_speed - self.agent.right_motor_speed) / self.agent.motor_offset
        adjusted_angle = math.radians(self.agent_angle)  # Convert angle to radians for trigonometric functions.

        if omega == 0:  # Special case: No rotation, straight movement.
            new_x = self.agent_pos.x + math.cos(adjusted_angle) * (self.agent.right_motor_speed + self.agent.left_motor_speed) / 2 * dt
            new_y = self.agent_pos.y + math.sin(adjusted_angle) * (self.agent.right_motor_speed + self.agent.left_motor_speed) / 2 * dt
        else:
            R = (self.agent.left_motor_speed + self.agent.right_motor_speed) / (2 * omega)  # Radius of curvature
            ICC_x = self.agent_pos.x - R * math.sin(adjusted_angle)
            ICC_y = self.agent_pos.y + R * math.cos(adjusted_angle)

            # Calculate new position based on rotation around the ICC.
            new_x = math.cos(omega * dt) * (self.agent_pos.x - ICC_x) - math.sin(omega * dt) * (self.agent_pos.y - ICC_y) + ICC_x
            new_y = math.sin(omega * dt) * (self.agent_pos.x - ICC_x) + math.cos(omega * dt) * (self.agent_pos.y - ICC_y) + ICC_y
            new_angle = self.agent_angle + math.degrees(omega * dt)  # Update orientation angle.

            self.agent_angle = new_angle % 360  # Normalize angle.
            self.robot_rotation()  # Update robot's graphical rotation.

        self.agent_pos = pygame.math.Vector2(new_x, new_y)  # Update robot's position.
        new_pos, self.collision = self.environment.detect_collision(old_pos, (new_x, new_y))
        self.agent_pos = pygame.math.Vector2(new_pos)  # Update to the collision-adjusted position.
        self.agent.sensors.agent_position = self.agent_pos  # Update sensor data with the new position.

    def robot_rotation(self):
        """
        Rotates the robot's image according to the current orientation angle for visual representation.
        Also updates the sensor's angle data.
        """
        try:
            self.agent.image = pygame.transform.rotate(self.agent.base_robot_image, -self.agent_angle)
            self.agent.rect = self.agent.image.get_rect(center=self.agent_pos)
        except:
            pass
        self.agent.sensors.agent_angle = self.agent_angle
