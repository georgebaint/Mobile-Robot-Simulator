import pygame
from sys import exit
import math
from settings import *
import numpy as np

class Robot(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.transform.rotozoom(pygame.image.load("images/robot.png").convert_alpha(), 0, ROBOT_SIZE)
        self.base_robot_image = self.image
        self.pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
        self.angle = 0

        self.speed = ROBOT_SPEED
        # Define the radius of the circular hitbox (eg half the width of the image)
        self.radius = self.base_robot_image.get_width() // 2
        self.rect = self.base_robot_image.get_rect(center=self.pos)

        # Parameters for motors
        self.right_motor_speed = 0
        self.left_motor_speed = 0
        self.motor_offset = self.radius*2

    def calculate_forward_kinematics(self):
        dt = 1  # Assuming the delta time is 1 frame.
        omega = (self.right_motor_speed - self.left_motor_speed) / self.motor_offset

        if omega == 0:  # Straight movement Exception
            # Adjust angle by subtracting 90 degrees for correct orientation 
            # Needs to be done so wheels are on the side of the robot since on default state it looks UP (90degrees) and not on the RIGHT(0degrees)
            adjusted_angle = math.radians(self.angle - 90)
            new_x = self.pos.x + math.cos(adjusted_angle) * (self.right_motor_speed + self.left_motor_speed) / 2 * dt
            new_y = self.pos.y + math.sin(adjusted_angle) * (self.right_motor_speed + self.left_motor_speed) / 2 * dt
            self.pos = pygame.math.Vector2(new_x, new_y)
        else:
            R = 1 / 2 * (self.left_motor_speed + self.right_motor_speed) / omega
            # Same adjustment for ICC calculations
            adjusted_angle = math.radians(self.angle - 90)
            ICC_x = self.pos.x - R * math.sin(adjusted_angle)
            ICC_y = self.pos.y + R * math.cos(adjusted_angle)

            new_x = math.cos(omega * dt) * (self.pos.x - ICC_x) - math.sin(omega * dt) * (self.pos.y - ICC_y) + ICC_x
            new_y = math.sin(omega * dt) * (self.pos.x - ICC_x) + math.cos(omega * dt) * (self.pos.y - ICC_y) + ICC_y
            new_angle = self.angle + math.degrees(omega * dt)

            self.pos = pygame.math.Vector2(new_x, new_y)
            self.angle = new_angle % 360  # Normalize angle

        # print(f"DEBUG: NEW_X = {new_x}, NEW_Y = {new_y}, ANGLE = {self.angle}")
        self.robot_rotation() 

    def robot_rotation(self):
        # Ensure image rotation matches the angle
        self.image = pygame.transform.rotate(self.base_robot_image, -self.angle)
        self.rect = self.image.get_rect(center=self.pos)                 

    def user_input(self):
        self.right_motor_speed = 0
        self.left_motor_speed = 0

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            self.left_motor_speed += -self.speed

        if keys[pygame.K_s]:
            self.left_motor_speed += self.speed

        if keys[pygame.K_a]:
            self.right_motor_speed += -self.speed

        if keys[pygame.K_d]:
            self.right_motor_speed += self.speed

        self.calculate_forward_kinematics()

        print(f"DEBUG: Left Motor Speed= {self.left_motor_speed}, Right Motor Speed= {self.right_motor_speed}")


    def update(self):
        self.user_input()
        self.robot_rotation()
        self.rect.center = self.pos

if __name__ == "__main__":
    pygame.init()

    # Creating the window
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption('Mobile Robot Simulation')
    clock = pygame.time.Clock()

    # Load images
    background = pygame.transform.scale(pygame.image.load("images/background.jpg").convert(), (WIDTH, HEIGHT))
    maze_image = pygame.transform.scale(pygame.image.load("images/maze.png").convert_alpha(), (WIDTH, HEIGHT))

    robot = Robot()

    while True:
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
        
        screen.blit(background, (0,0))
        background.blit(maze_image, (0,0))
        screen.blit(robot.image, robot.rect)
        robot.update()

        pygame.draw.circle(screen, "blue", (int(robot.pos.x), int(robot.pos.y)), robot.radius, width=2)
        pygame.draw.rect(screen, "green", robot.rect, width=2)

        pygame.display.update()
        clock.tick(FPS)