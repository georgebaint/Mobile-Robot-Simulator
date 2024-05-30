from agent import Agent
from environment import Environment
from forward_kinematics import ForwardKinematics

import pygame
import cv2 as cv
import numpy as np
import time
from tqdm import tqdm

from settings import *

class DustingSimulation:
    def __init__(self, maze_id, genotype=None):        
                
        maze = cv.imread('images/maze/m%d.png' % (maze_id), cv.IMREAD_GRAYSCALE)
        self.pixel_map = cv.resize(maze, (WIDTH, HEIGHT), interpolation=cv.INTER_NEAREST)
        self.screen = None

        if VISUALIZE:
            pygame.init()
            self.clock = pygame.time.Clock()
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
            self.maze_surface = pygame.surfarray.make_surface(self.pixel_map.swapaxes(0, 1))
        
        self.environment = Environment(maze_id)
        self.agent = Agent(self.environment)
        self.forward_kinematics = ForwardKinematics(self.agent, self.environment)
        
        self.genotype = genotype

    def run_ann(self, iter, ann):
        score = 0
        collision_cnt = 0
        max_score = self.environment.get_max_score()

        for i in tqdm(range(iter)):
            score += self.environment.suck(self.forward_kinematics.agent_pos)

            self.forward_kinematics.calculate_forward_kinematics()
            if self.forward_kinematics.collision:
                collision_cnt += 1

            # self.agent.kalman_filter(self.forward_kinematics)
            # Next two lines replace kalman filter output with real position
            self.agent.estimated_pos = self.forward_kinematics.agent_pos
            self.agent.estimated_angle = self.forward_kinematics.agent_angle

            distances = self.agent.run_sensors(self.screen)
            input = [i / MAX_SENSOR_LENGTH for i in distances]
            # print("input = ", input)

            input.append(self.agent.left_motor_speed / ROBOT_MAX_SPEED) 
            input.append(self.agent.right_motor_speed / ROBOT_MAX_SPEED)

            spds = ann.calculate_output(input)

            self.agent.left_motor_speed = spds[0] * ROBOT_MAX_SPEED
            self.agent.right_motor_speed = spds[1] *  ROBOT_MAX_SPEED

        
            if VISUALIZE:
                self.screen.blit(self.maze_surface, (0,0))
                self.agent.rect.center = self.forward_kinematics.agent_pos

                self.environment.draw_landmarks(self.screen)
                self.screen.blit(self.agent.image, self.agent.rect)
                # if prev_cc != [-1, -1]:
                #     pygame.draw.circle(self.screen, 'green', (int(prev_cc.x), int(prev_cc.y)), self.agent.radius, 2)        
                pygame.draw.circle(self.screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
                
                # Display the current wheel speeds
                self.draw_text(self.screen, f"Left Wheel Speed: {self.agent.left_motor_speed:.2f}", (10, 10))
                self.draw_text(self.screen, f"Right Wheel Speed: {self.agent.right_motor_speed:.2f}", (10, 40))

                self.draw_text(self.screen, f"Real position {self.forward_kinematics.agent_pos[0]:.0f}, {self.forward_kinematics.agent_pos[1]:.0f}", (1000,570))
                self.draw_text(self.screen, f"Real angle {self.forward_kinematics.agent_angle:.0f}", (1000,590))
                self.draw_text(self.screen, f"Estimated position {self.agent.estimated_pos[0]:.0f}, {self.agent.estimated_pos[1]:.0f}", (1000,610))
                self.draw_text(self.screen, f"Estimated angle {self.agent.estimated_angle:.0f}", (1000,630))

                self.draw_text(self.screen, f"Dust cleared: {(float(score) / float(max_score) * 100):.2f}%", (1000,530))
                self.draw_text(self.screen, f"Collision rate : {float(collision_cnt) / float(iter) * 100:.2f}%", (1000,550))

                
                pygame.display.update()
                self.clock.tick(FPS)

        dust_rate = (float(score) / float(max_score))
        collision_rate = float(collision_cnt) / float(iter) 
        return 1 * dust_rate - 0.2 * collision_rate

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
            
            if VISUALIZE:
                self.screen.blit(self.maze_surface, (0,0))

            self.forward_kinematics.calculate_forward_kinematics()
            self.agent.kalman_filter(self.forward_kinematics)
            distances = self.agent.run_sensors(self.screen)
        
            if VISUALIZE:
                self.agent.rect.center = self.forward_kinematics.agent_pos

                self.environment.draw_landmarks(self.screen)
                self.screen.blit(self.agent.image, self.agent.rect)
                if prev_cc != [-1, -1]:
                    pygame.draw.circle(self.screen, 'green', (int(prev_cc.x), int(prev_cc.y)), self.agent.radius, 2)        
                pygame.draw.circle(self.screen, ('blue' if not self.agent.collision else 'yellow'), (int(self.forward_kinematics.agent_pos.x), int(self.forward_kinematics.agent_pos.y)), self.agent.radius, width=2)
                
                # Display the current wheel speeds
                self.draw_text(self.screen, f"Left Wheel Speed: {self.agent.left_motor_speed:.2f}", (10, 10))
                self.draw_text(self.screen, f"Right Wheel Speed: {self.agent.right_motor_speed:.2f}", (10, 40))

                self.draw_text(self.screen, f"Real position {self.forward_kinematics.agent_pos[0]:.0f}, {self.forward_kinematics.agent_pos[1]:.0f}", (1000,570))
                self.draw_text(self.screen, f"Real angle {self.forward_kinematics.agent_angle:.0f}", (1000,590))
                self.draw_text(self.screen, f"Estimated position {self.agent.estimated_pos[0]:.0f}, {self.agent.estimated_pos[1]:.0f}", (1000,610))
                self.draw_text(self.screen, f"Estimated angle {self.agent.estimated_angle:.0f}", (1000,630))
                
                pygame.display.update()
                self.clock.tick(FPS)
        print(changes)
        return score
    
    def draw_text(self, screen, text, position, font_size=24, color='red'):
        """
        Draws text on the screen at the specified position.

        Args:
            screen (Surface): The Pygame surface where text will be drawn.
            text (str): The text to be drawn.
            position (tuple): The (x, y) position for the text on the screen.
            font_size (int): The font size for the text.
            color (str): The color of the text.
        """
        font = pygame.font.Font(None, font_size)  # None uses the default font, set font path for custom font
        text_surface = font.render(text, True, color)  # True means anti-aliased text.
        screen.blit(text_surface, position)

if __name__ == '__main__':
    sim = DustingSimulation(MAZE_NUM, None)
    score = sim.evaluate(ITER_COUNT)
    print('final score %d' % (score))

# TODO

# prepare the training pipeline:

# feed sensor data to ANN and reset the speed on the wheel
# add dust and dust detection, punish collisions
# fitness = (dust collected/dust total) * w1 + (num of states with no collision/total iters) * w2 (w1=1, w2=0.2)?

# normalize/scale everything (sensor distances, scores, think about maze and robot size, starting positions):
# keep sensor distances the same (maybe substract radius, optionally scale to [0,1])
# the score should mean the same thing for all simulations
# robot size matters since it impacts sensors and subsequent motion => robot size stays the same
# maze size can differ but might not be optimal for model convergence

