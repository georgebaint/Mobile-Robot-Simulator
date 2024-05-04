import pygame
import math
from settings import *
import matplotlib.pyplot as plt
import numpy as np

class Agent:
    class Sensors:
        def __init__(self):
            self.agent_postion = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
            self.agent_angle = 0
        
        def run(self, agent, screen):
            num_sensors = 12
            max_sensor_length = 200
            tab20 = plt.get_cmap("tab20").colors
            font = pygame.font.Font(None, 24)

            for i in range(num_sensors):
                angle = math.radians((360 / num_sensors) * i + self.agent_angle)
                for distance in range(agent.radius, agent.radius + max_sensor_length):
                    # print(distance)
                    end_x = int(round(self.agent_pos.x + distance * math.cos(angle)))
                    end_y = int(round(self.agent_pos.y + distance * math.sin(angle)))
                    #TODO if agent.evironment.is_landmark(end_x, end_y): 
                        # use this landmark for the agent 
                        # update this landmark's flag to true
                    if agent.environment.is_wall(end_x, end_y):
                        break
                else:
                    distance = agent.radius + max_sensor_length  # Ensure distance is set if no wall is hit

                start_pos = (int(self.agent_pos.x + agent.radius * math.cos(angle)), int(self.agent_pos.y + agent.radius * math.sin(angle)))
                end_pos = (int(self.agent_pos.x + distance * math.cos(angle)), int(self.agent_pos.y + distance * math.sin(angle)))
                color = tuple(int(255 * x) for x in tab20[i % len(tab20)])
                pygame.draw.line(screen, color, start_pos, end_pos, 2)

                # Display the distance
                distance_text = font.render(f"{distance - agent.radius}", True, (255, 0, 255))
                screen.blit(distance_text, end_pos)
                # print(start_pos)

    def __init__(self, environment):
        self.environment = environment
        
        self.image = pygame.transform.rotozoom(pygame.image.load("images/robot.png").convert_alpha(), 0, ROBOT_SIZE)
        self.base_robot_image = self.image

        # self.pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
        self.estimated_pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)

        # self.angle = 0
        self.estimated_angle = 0

        self.collision = False

        # self.speed = ROBOT_MAX_SPEED
        # Define the radius of the circular hitbox (eg half the width of the image)
        self.radius = self.base_robot_image.get_width() // 2
        self.rect = self.base_robot_image.get_rect(center=pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y))

        # Parameters for motors
        self.right_motor_speed = 0
        self.left_motor_speed = 0
        self.motor_offset = self.radius*2

        self.sensors = self.Sensors()

        self.mu = np.zeros(3)
        self.sigma = np.eye(3)

    def run_sensors(self, screen):
        self.sensors.run(self, screen)
    
    # def calculate_forward_kinematics(self,):
    #     old_pos = (self.pos.x, self.pos.y)

    #     dt = 1  # Assuming the delta time is 1 frame.
    #     omega = (self.left_motor_speed - self.right_motor_speed) / self.motor_offset
    #     adjusted_angle = math.radians(self.angle)

    #     if omega == 0:  # Straight movement Exception
    #         new_x = self.pos.x + math.cos(adjusted_angle) * (self.right_motor_speed + self.left_motor_speed) / 2 * dt
    #         new_y = self.pos.y + math.sin(adjusted_angle) * (self.right_motor_speed + self.left_motor_speed) / 2 * dt
    #         self.pos = pygame.math.Vector2(new_x, new_y)
            
    #     else:
    #         R = 1 / 2 * (self.left_motor_speed + self.right_motor_speed) / omega

    #         ICC_x = self.pos.x - R * math.sin(adjusted_angle)
    #         ICC_y = self.pos.y + R * math.cos(adjusted_angle)

    #         new_x = math.cos(omega * dt) * (self.pos.x - ICC_x) - math.sin(omega * dt) * (self.pos.y - ICC_y) + ICC_x
    #         new_y = math.sin(omega * dt) * (self.pos.x - ICC_x) + math.cos(omega * dt) * (self.pos.y - ICC_y) + ICC_y
    #         new_angle = self.angle + math.degrees(omega * dt)

    #         self.pos = pygame.math.Vector2(new_x, new_y)
    #         self.angle = new_angle % 360  # Normalize angle

    #         self.robot_rotation() 
        
    #     new_pos = (self.pos.x, self.pos.y)
    #     new_pos, self.collision = self.environment.detect_collision(old_pos, new_pos, )
    #     self.pos = pygame.math.Vector2(new_pos)
    #     # print(f"DEBUG: NEW_X = {new_x}, NEW_Y = {new_y}, ANGLE = {self.angle}")

    # def robot_rotation(self):
    #     # Ensures image rotates with the angle
    #     self.image = pygame.transform.rotate(self.base_robot_image, -self.angle)
    #     self.rect = self.image.get_rect(center=self.pos)

    # def update_image(self,):

    def kalman_filter(self, forward_kinematics):

        dt = 1
        omega = (self.left_motor_speed - self.right_motor_speed) / self.motor_offset
        velocity = (self.left_motor_speed + self.right_motor_speed) / 2
        A = np.eye(3)
        B = np.array([[dt*np.cos(self.estimated_angle), 0],
             [dt*np.sin(self.estimated_angle), 0],
             [0, dt]])
        
        u = np.array([omega, velocity]).T

        sigma_Rx = 3
        sigma_Ry = 3
        sigma_Rtheta = 5

        R = np.array([[sigma_Rx**2, 0, 0],
                        [0, sigma_Ry**2, 0],
                        [0, 0, sigma_Rtheta**2]])

        C = np.eye(3)
        sigma_Qx = 1
        sigma_Qy = 1
        sigma_Qtheta = 2

        Q = np.array([[sigma_Qx**2, 0, 0],
                        [0, sigma_Qy**2, 0],
                        [0, 0, sigma_Qtheta**2]])

        # epsilon = np.random.multivariate_normal(np.array([0,0,0]),R, 1)

        # new_pos = A * self.pos + B * u + epsilon
        
        #Prediction
        mu_prediction = A @ self.mu + B @ u
        sigma_prediction = A @ self.sigma @ A.T + R

        #TODO: z = detect_landmarks...
        # z = np.zeros(3)
        z = np.array([forward_kinematics.agent_pos.x, forward_kinematics.agent_pos.y, forward_kinematics.agent_angle])


        # z = mu_prediction
        #Correction
        K = sigma_prediction @ C.T @ np.linalg.inv(C @ sigma_prediction @ C.T + Q)
        mu = mu_prediction + K @ (z - C @ mu_prediction)
        sigma = (np.eye(3) - K @ C) @ sigma_prediction

        new_pos = np.random.multivariate_normal(mu, sigma, 1)
        self.estimated_pos = new_pos

        self.mu = mu
        self.sigma = sigma
