#from mobile_robot import Agent, Environemnt, ForwardKinematics
from mobile_robot import Agent

# from agent import Agent
# from environment import Environment
# from forward_kinematics import ForwardKinematics


from controls import Controls
from settings import *

import pygame
import cv2 as cv
import numpy as np

HEIGHT = 1280
HEIGHT = 720

class DustingSimulation:
    def __init__(self, genotype=None, visualize=False):        
        self.environment = Environment()
        self.agent = Agent(self.environment)
        self.forward_kinematics = ForwardKinematics(self.agent, self.environment)
        
        background = cv.imread('images/background.jpg')
        self.background = cv.resize(background, (WIDTH, HEIGHT))
        
        maze = cv.imread('images/maze.png', cv.IMREAD_GRAYSCALE)
        _, pixel_map = cv.threshold(maze, 127, 255, cv.THRESH_BINARY)
        self.pixel_map = cv.resize(pixel_map, (WIDTH, HEIGHT))
        
        self.genotype = genotype
        self.visualize = visualize

        if visualize:
            pygame.init()
            self.clock = pygame.time.Clock()
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
            self.background_surface = pygame.surfarray.make_surface(background.swapaxes(0, 1))
            self.maze_surface = pygame.surfarray.make_surface(self.pixel_map.swapaxes(0, 1)) 

    def evaluate(self, iter):
        change_counter=100

        score = 0
        for i in range(iter):
            change_counter -= 1
            if change_counter == 0:
                spd = [[1, 1], [-1, -1], [1, -1]]
                np.random.shuffle(spd)
                self.agent.left_motor_speed, self.agent.right_motor_speed = spd[0], spd[1]
                change_counter = 100

            if self.visualize:
                self.screen.blit(self.background, (0,0))
                self.screen.blit(self.maze_surface, (0,0))
            
            self.forward_kinematics.calculate_forward_kinematics()

            if self.visualize:
                self.screen.blit(self.agent.image, self.agent.rect)
                pygame.draw.circle(self.screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
                pygame.display.update()
                self.clock.tick(FPS)
    
        return score

if __name__ == '__main__':
    sim = DustingSimulation(None, True)
    score = sim.evaluate(1000)
    print('final score %d' % (score))