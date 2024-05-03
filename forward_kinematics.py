import numpy as np
import math
import pygame
from settings import *

class ForwardKinematics:
    def __init__(self, agent, environment):
        self.agent = agent
        self.agent_pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
        self.agent_angle = 0
        self.environment = environment
        self.landmarks = environment.landmarks



    def calculate_forward_kinematics(self,):
        old_pos = (self.agent_pos.x, self.agent_pos.y)

        dt = 1  # Assuming the delta time is 1 frame.
        omega = (self.agent.left_motor_speed - self.agent.right_motor_speed) / self.agent.motor_offset
        adjusted_angle = math.radians(self.agent_angle)

        if omega == 0:  # Straight movement Exception
            new_x = self.agent_pos.x + math.cos(adjusted_angle) * (self.agent.right_motor_speed + self.agent.left_motor_speed) / 2 * dt
            new_y = self.agent_pos.y + math.sin(adjusted_angle) * (self.agent.right_motor_speed + self.agent.left_motor_speed) / 2 * dt
            self.agent_pos = pygame.math.Vector2(new_x, new_y)
            
        else:
            R = 1 / 2 * (self.agent.left_motor_speed + self.agent.right_motor_speed) / omega

            ICC_x = self.agent_pos.x - R * math.sin(adjusted_angle)
            ICC_y = self.agent_pos.y + R * math.cos(adjusted_angle)

            new_x = math.cos(omega * dt) * (self.agent_pos.x - ICC_x) - math.sin(omega * dt) * (self.agent_pos.y - ICC_y) + ICC_x
            new_y = math.sin(omega * dt) * (self.agent_pos.x - ICC_x) + math.cos(omega * dt) * (self.agent_pos.y - ICC_y) + ICC_y
            new_angle = self.agent_angle + math.degrees(omega * dt)

            self.agent_pos = pygame.math.Vector2(new_x, new_y)
            self.agent_angle = new_angle % 360  # Normalize angle

            self.robot_rotation() 
        
        new_pos = (self.agent_pos.x, self.agent_pos.y)
        new_pos, self.collision = self.environment.detect_collision(old_pos, new_pos)
        self.agent_pos = pygame.math.Vector2(new_pos)
        # print(self.agent_pos)
        # print(f"DEBUG: NEW_X = {new_x}, NEW_Y = {new_y}, ANGLE = {self.angle}")

    def robot_rotation(self):
        # Ensures image rotates with the angle
        self.agent.image = pygame.transform.rotate(self.agent.base_robot_image, -self.agent_angle)
        self.agent.rect = self.agent.image.get_rect(center=self.agent_pos)
