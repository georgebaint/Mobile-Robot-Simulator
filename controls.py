import pygame
from settings import *
import sys

class Controls:
    """
    Manages user input for adjusting the motor speeds of the robot.

    Attributes:
        speed_on_hold (bool): A flag to maintain the last set speeds if no new input is detected.
    """
    def __init__(self):
        self.speed_on_hold = True
        
    def user_input(self, agent):
        """
        Processes key inputs from the user to control the robotic agent's motors and handle system commands.

        Args:
            agent (Agent): The robot agent being controlled.
        
        Returns:
            bool: True if a snapshot should be taken (when 'N' is pressed), False otherwise.
        """
        keys = pygame.key.get_pressed()
        
        any_key_pressed = False

        # Individual key effects
        if keys[pygame.K_q]:
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.left_motor_speed + SPEED_DELTA))
            any_key_pressed = True
        if keys[pygame.K_a]:
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.left_motor_speed - SPEED_DELTA))
            any_key_pressed = True
        if keys[pygame.K_p]:
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.right_motor_speed + SPEED_DELTA))
            any_key_pressed = True
        if keys[pygame.K_l]:
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.right_motor_speed - SPEED_DELTA))
            any_key_pressed = True

        # Handling simultaneous arrow key presses for coordinated movement
        if keys[pygame.K_UP]:
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.left_motor_speed + SPEED_DELTA))
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.right_motor_speed + SPEED_DELTA))
            any_key_pressed = True
        if keys[pygame.K_DOWN]:
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.left_motor_speed - SPEED_DELTA))
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.right_motor_speed - SPEED_DELTA))
            any_key_pressed = True

        if keys[pygame.K_RIGHT]:
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.right_motor_speed - ROTATION_SPEED_DELTA))
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.left_motor_speed + ROTATION_SPEED_DELTA))
            any_key_pressed = True
        if keys[pygame.K_LEFT]:
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.right_motor_speed + ROTATION_SPEED_DELTA))
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, min(ROBOT_MAX_SPEED, agent.left_motor_speed - ROTATION_SPEED_DELTA))
            any_key_pressed = True
        
        if keys[pygame.K_r]:
            agent.left_motor_speed = min(ROBOT_MAX_SPEED, agent.left_motor_speed + ROTATION_SPEED_DELTA)
            agent.right_motor_speed = min(ROBOT_MAX_SPEED, agent.right_motor_speed + ROTATION_SPEED_DELTA)
        if keys[pygame.K_f]:
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, agent.left_motor_speed - ROTATION_SPEED_DELTA) 
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, agent.right_motor_speed - ROTATION_SPEED_DELTA)
        if keys[pygame.K_d]:
            agent.left_motor_speed = min(ROBOT_MAX_SPEED, agent.left_motor_speed + ROTATION_SPEED_DELTA)
            agent.right_motor_speed = max(-ROBOT_MAX_SPEED, agent.right_motor_speed - ROTATION_SPEED_DELTA)
        if keys[pygame.K_g]:
            agent.left_motor_speed = max(-ROBOT_MAX_SPEED, agent.left_motor_speed - ROTATION_SPEED_DELTA)
            agent.right_motor_speed = min(ROBOT_MAX_SPEED, agent.right_motor_speed + ROTATION_SPEED_DELTA)
        if keys[pygame.K_v]:
            agent.left_motor_speed = 0
            agent.right_motor_speed = 0
        
        if keys[pygame.K_m]:
            self.speed_on_hold ^= 1
        
        # If no keys that affect motor speed were pressed, reset the motor speeds to zero
        if (not any_key_pressed) and self.speed_on_hold:
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