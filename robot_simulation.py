import pygame
from sys import exit
import math
from settings import *


class Robot(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.transform.rotozoom(pygame.image.load("images/robot.png").convert_alpha(), 0, ROBOT_SIZE)
        self.base_robot_image = self.image
        self.pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
        self.speed = ROBOT_SPEED
        # Define the radius of the circular hitbox (eg half the width of the image)
        self.radius = self.base_robot_image.get_width() // 2
        self.rect = self.base_robot_image.get_rect(center=self.pos)

    def robot_rotation(self):
        self.angle = math.degrees(math.atan2(self.velocity_x, -self.velocity_y))
        self.image = pygame.transform.rotate(self.base_robot_image, -self.angle)
        self.rect = self.image.get_rect(center=self.pos)

    def user_input(self):
        # TODO Velocity of different wheels
        self.velocity_x = 0
        self.velocity_y = 0

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            self.velocity_y = -self.speed

        if keys[pygame.K_s]:
            self.velocity_y = self.speed

        if keys[pygame.K_a]:
            self.velocity_x = -self.speed

        if keys[pygame.K_d]:
            self.velocity_x = self.speed

        if self.velocity_x != 0 and self.velocity_y != 0:  # Robot is moving diagonally
            self.velocity_x /= math.sqrt(2)  # From Pythagorean theorem
            self.velocity_y /= math.sqrt(2)

    def move(self):
        self.pos += pygame.math.Vector2(self.velocity_x, self.velocity_y)
        self.rect.center = self.pos

    def update(self):
        self.user_input()
        self.move()
        self.robot_rotation()


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