import pygame
from settings import *

def user_input(agent):
    keys = pygame.key.get_pressed()
    
    if keys[pygame.K_q]:
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed + 0.2))  # Increase speed of left motor with Q
    if keys[pygame.K_a]:
        agent.left_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.left_motor_speed - 0.2))  # Decrease speed of left motor with A
    if keys[pygame.K_p]:
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed + 0.2))  # Increase speed of right motor with P
    if keys[pygame.K_l]:
        agent.right_motor_speed = max(-ROBOT_SPEED, min(ROBOT_SPEED, agent.right_motor_speed - 0.2))  # Decrease speed of right motor with L
    
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
