import pygame
from settings import *
import sys

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
    
    if keys[pygame.K_r]:
        agent.left_motor_speed = min(ROBOT_SPEED, agent.left_motor_speed + 0.2)
        agent.right_motor_speed = min(ROBOT_SPEED, agent.right_motor_speed + 0.2)
    if keys[pygame.K_f]:
        agent.left_motor_speed = max(-ROBOT_SPEED, agent.left_motor_speed - 0.2) 
        agent.right_motor_speed = max(-ROBOT_SPEED, agent.right_motor_speed - 0.2)
    if keys[pygame.K_d]:
        agent.left_motor_speed = min(ROBOT_SPEED, agent.left_motor_speed + 0.2)
        agent.right_motor_speed = min(ROBOT_SPEED, agent.right_motor_speed - 0.2)
    if keys[pygame.K_g]:
        agent.left_motor_speed = min(ROBOT_SPEED, agent.left_motor_speed - 0.2)
        agent.right_motor_speed = min(ROBOT_SPEED, agent.right_motor_speed + 0.2)
    if keys[pygame.K_v]:
        agent.left_motor_speed = 0
        agent.right_motor_speed = 0
    
    if keys[pygame.K_ESCAPE]:
        pygame.display.quit()
        pygame.quit()
        sys.exit()

    take_snapshot = False
    if keys[pygame.K_n]:
        take_snapshot = True
    return take_snapshot