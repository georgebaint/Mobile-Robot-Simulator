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
ITER_COUNT = 1600
COUNTER_DROP = 200
MAZE_NUM = 2

class DustingSimulation:
    def __init__(self, maze_id, genotype=None, visualize=False):        
                
        maze = cv.imread('images/maze/m%d.png' % (maze_id), cv.IMREAD_GRAYSCALE)
        self.pixel_map = cv.resize(maze, (WIDTH, HEIGHT), interpolation=cv.INTER_NEAREST)
        
        if visualize:
            pygame.init()
            self.clock = pygame.time.Clock()
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
            self.maze_surface = pygame.surfarray.make_surface(self.pixel_map.swapaxes(0, 1))
        
        self.environment = Environment(maze_id)
        self.agent = Agent(self.environment)
        self.forward_kinematics = ForwardKinematics(self.agent, self.environment)
        
        self.genotype = genotype
        self.visualize = visualize

    def evaluate(self, iter):
        change_counter = COUNTER_DROP

        score = 0
        changes = []
        poss = []
        prev_cc = [-1, -1]
        for i in tqdm(range(iter)):
            change_counter -= 1
            if change_counter == 0:
                score_change = self.environment.suck(self.forward_kinematics.agent_pos)
                score += score_change
                changes.append(score_change)
                prev_cc = self.forward_kinematics.agent_pos
                spd = [[1, 1], [-1, -1], [1, -1], [-1, 0], [0, 1]]
                spd = np.array(spd)
                np.random.shuffle(spd)
                self.agent.left_motor_speed, self.agent.right_motor_speed = spd[0,0], spd[0,1]
                change_counter = COUNTER_DROP
            
            self.forward_kinematics.calculate_forward_kinematics()
        
            if self.visualize:
                self.agent.rect.center = self.forward_kinematics.agent_pos

                self.screen.blit(self.maze_surface, (0,0))
                self.screen.blit(self.agent.image, self.agent.rect)
                if prev_cc != [-1, -1]:
                    pygame.draw.circle(self.screen, 'green', (int(prev_cc.x), int(prev_cc.y)), self.agent.radius, 2)        
                pygame.draw.circle(self.screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
                pygame.display.update()
                self.clock.tick(FPS)
        print(changes)
        return score

if __name__ == '__main__':
    sim = DustingSimulation(MAZE_NUM, None, True)
    score = sim.evaluate(ITER_COUNT)
    print('final score %d' % (score))

# TODO

# check if positions, speeds, etc are all correct

# make dust visualization with sucking on pressed key

# connect the Kalman filter

# prepare the training pipeline:

# call sensors on each step
# feed sensor data to ANN and reset the speed on the wheel
# add dust and dust detection, punish collisions
# fitness = (dust collected/dust total) * w1 + (num of states with no collision/total iters) * w2 (w1=1, w2=0.2)?

# normalize/scale everything (sensor distances, scores, think about maze and robot size, starting positions):
# keep sensor distances the same (maybe substract radius, optionally scale to [0,1])
# the score should mean the same thing for all simulations
# robot size matters since it impacts sensors and subsequent motion => robot size stays the same
# maze size can differ but might not be optimal for model convergence

