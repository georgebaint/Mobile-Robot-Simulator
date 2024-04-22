import pygame
from settings import *
import sys

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


    
    if keys[pygame.K_r]:
        agent.left_motor_speed = min(ROBOT_SPEED, agent.left_motor_speed + ROTATION_SPEED_DELTA)
        agent.right_motor_speed = min(ROBOT_SPEED, agent.right_motor_speed + ROTATION_SPEED_DELTA)
    if keys[pygame.K_f]:
        agent.left_motor_speed = max(-ROBOT_SPEED, agent.left_motor_speed - ROTATION_SPEED_DELTA) 
        agent.right_motor_speed = max(-ROBOT_SPEED, agent.right_motor_speed - ROTATION_SPEED_DELTA)
    if keys[pygame.K_d]:
        agent.left_motor_speed = min(ROBOT_SPEED, agent.left_motor_speed + ROTATION_SPEED_DELTA)
        agent.right_motor_speed = min(ROBOT_SPEED, agent.right_motor_speed - ROTATION_SPEED_DELTA)
    if keys[pygame.K_g]:
        agent.left_motor_speed = min(ROBOT_SPEED, agent.left_motor_speed - ROTATION_SPEED_DELTA)
        agent.right_motor_speed = min(ROBOT_SPEED, agent.right_motor_speed + ROTATION_SPEED_DELTA)
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