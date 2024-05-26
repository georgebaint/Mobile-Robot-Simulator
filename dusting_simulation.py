from agent import Agent
from environment import Environment
from forward_kinematics import ForwardKinematics

import pygame
import cv2 as cv
import numpy as np
import time
from tqdm import tqdm

WIDTH = 1280
HEIGHT = 720
FPS = 120

class DustingSimulation:
    def __init__(self, genotype=None, visualize=False):        
        
        background = cv.imread('images/background.jpg')
        self.background = cv.resize(background, (WIDTH, HEIGHT))
        
        maze = cv.imread('images/maze.png', cv.IMREAD_GRAYSCALE)
        _, pixel_map = cv.threshold(maze, 127, 255, cv.THRESH_BINARY)
        self.pixel_map = cv.resize(pixel_map, (WIDTH, HEIGHT))
        
        if visualize:
            pygame.init()
            self.clock = pygame.time.Clock()
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
            self.background_surface = pygame.surfarray.make_surface(background.swapaxes(0, 1))
            self.maze_surface = pygame.surfarray.make_surface(self.pixel_map.swapaxes(0, 1))
        
        self.environment = Environment()
        self.agent = Agent(self.environment)
        self.forward_kinematics = ForwardKinematics(self.agent, self.environment)
        
        self.genotype = genotype
        self.visualize = visualize

    def evaluate(self, iter):
        change_counter = 100

        score = 0
        for i in tqdm(range(iter)):
            change_counter -= 1
            if change_counter == 0:
                spd = [[1, 1], [-1, -1], [1, -1]]
                spd = np.array(spd)
                np.random.shuffle(spd)
                self.agent.left_motor_speed, self.agent.right_motor_speed = spd[0,0], spd[0,1]
                change_counter = 100
            
            self.forward_kinematics.calculate_forward_kinematics()
        
            if self.visualize:
                self.agent.rect.center = self.forward_kinematics.agent_pos

                self.screen.blit(self.background_surface, (0,0))
                self.screen.blit(self.maze_surface, (0,0))
                self.screen.blit(self.agent.image, self.agent.rect)
                pygame.draw.circle(self.screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
                pygame.display.update()
                self.clock.tick(FPS)
        return score

if __name__ == '__main__':
    sim = DustingSimulation(None, True)
    score = sim.evaluate(1000)
    print('final score %d' % (score))

#TODO
# call sensors on each step
# feed sensor data to ANN and reset the speed on the wheel
# add dust and dust detection, punish collisions
# fitness = (dust collected/dust total) * w1 + (num of states with no collision/total iters) * w2 (w1=1, w2=0.2)?

# normalize/scale everything (sensor distances, scores, think about maze and robot size, starting positions):
# keep sensor distances the same (maybe substract radius, optionally scale to [0,1])
# the score should mean the same thing for all simulations
# robot size matters since it impacts sensors and subsequent motion => robot size stays the same
# maze size can differ but might not be optimal for model convergence