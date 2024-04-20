import pygame
from settings import *

import pygame

import pygame

import pygame

def user_input(agent):
    keys = pygame.key.get_pressed()
    
    any_key_pressed = False

    # Individual key effects
    if keys[pygame.K_q]:
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed + 0.2))
        any_key_pressed = True
    if keys[pygame.K_a]:
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed - 0.2))
        any_key_pressed = True
    if keys[pygame.K_p]:
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed + 0.2))
        any_key_pressed = True
    if keys[pygame.K_l]:
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed - 0.2))
        any_key_pressed = True

    # Handling simultaneous arrow key presses for coordinated movement
    if keys[pygame.K_UP]:
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed + 0.2))
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed + 0.2))
        any_key_pressed = True
    if keys[pygame.K_DOWN]:
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed - 0.2))
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed - 0.2))
        any_key_pressed = True

    if keys[pygame.K_RIGHT]:
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed - 0.3))
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed + 0.3))
        any_key_pressed = True
    if keys[pygame.K_LEFT]:
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed + 0.3))
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed - 0.3))
        any_key_pressed = True

    # If no keys that affect motor speed were pressed, reset the motor speeds to zero
    if not any_key_pressed:
        agent.left_motor_speed = 0
        agent.right_motor_speed = 0


    
    # print(f"DEBUG: Left Motor Speed= {self.left_motor_speed}, Right Motor Speed= {self.right_motor_speed}")

# def user_input(self):
#     self.right_motor_speed = 0
#     self.left_motor_speed = 0

#     keys = pygame.key.get_pressed()

#     if keys[pygame.K_w]:
#         self.left_motor_speed += -self.speed

#     if keys[pygame.K_s]:
#         self.left_motor_speed += self.speed

#     if keys[pygame.K_a]:
#         self.right_motor_speed += -self.speed

#     if keys[pygame.K_d]:
#         self.right_motor_speed += self.speed

#     self.calculate_forward_kinematics()

#     print(f"DEBUG: Left Motor Speed= {self.left_motor_speed}, Right Motor Speed= {self.right_motor_speed}")
