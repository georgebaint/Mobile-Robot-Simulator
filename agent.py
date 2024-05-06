import pygame
import math
from settings import *
import matplotlib.pyplot as plt
import numpy as np

class Agent:
    """
    Represents our robotic agent with sensor and movement capabilities in a simulated environment.
    This class handles the robot's image rendering, collision detection, and navigation logic.
    
    Attributes:
        environment (Environment): The simulation environment.
        image (Surface): The current image of the robot, rotated to its current angle.
        base_robot_image (Surface): The original image of the robot.
        estimated_pos (pygame.math.Vector2): The estimated position of the robot.
        estimated_angle (float): The estimated orientation angle of the robot in degrees.
        collision (bool): Flag indicating whether the robot has collided with an obstacle.
        radius (int): Radius of the robot's circular hitbox.
        rect (Rect): The bounding rectangle of the robot's image.
        right_motor_speed (float): Speed of the right motor.
        left_motor_speed (float): Speed of the left motor.
        motor_offset (float): Distance between the motors, affecting turning radius.
        sensors (Sensors): Sensor system of the robot.
        mu (numpy.ndarray): Mean state vector for Kalman filter.
        sigma (numpy.ndarray): Covariance matrix for Kalman filter.
    """

    class Sensors:
        """
        Nested class for the sensors of the robot.

        Attributes:
            agent_position (pygame.math.Vector2): The current position of the robot.
            agent_angle (float): The current orientation angle of the robot in degrees.
        """
        def __init__(self):
            self.agent_position = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
            self.agent_angle = 0
        
        def run(self, agent, screen):
            """
            Executes sensor processing to detect walls and draw beams on the screen.

            Args:
                agent (Agent): The robot agent using the sensors.
                screen (Surface): The Pygame screen surface to draw sensor beams.
            """
            num_sensors = 12
            max_sensor_length = 200
            tab20 = plt.get_cmap("tab20").colors  # Color map for sensor visualization.
            font = pygame.font.Font(None, 24)

            for i in range(num_sensors):
                angle = math.radians((360 / num_sensors) * i + self.agent_angle)
                for distance in range(agent.radius, agent.radius + max_sensor_length):
                    end_x = int(round(self.agent_position.x + distance * math.cos(angle)))
                    end_y = int(round(self.agent_position.y + distance * math.sin(angle)))
                    if agent.environment.is_wall(end_x, end_y):
                        break
                else:
                    distance = agent.radius + max_sensor_length

                start_pos = (int(self.agent_position.x + agent.radius * math.cos(angle)), int(self.agent_position.y + agent.radius * math.sin(angle)))
                end_pos = (int(self.agent_position.x + distance * math.cos(angle)), int(self.agent_position.y + distance * math.sin(angle)))
                color = tuple(int(255 * x) for x in tab20[i % len(tab20)])
                pygame.draw.line(screen, color, start_pos, end_pos, 2)
                distance_text = font.render(f"{distance - agent.radius}", True, (255, 0, 255))
                screen.blit(distance_text, end_pos)

    def __init__(self, environment):
        """
        Initializes an Agent within the specified environment.
        """

        self.environment = environment
        
        self.image = pygame.transform.rotozoom(pygame.image.load("images/robot.png").convert_alpha(), 0, ROBOT_SIZE)
        self.base_robot_image = self.image
        self.estimated_pos = pygame.math.Vector2(ROBOT_START_X, ROBOT_START_Y)
        self.estimated_angle = 0
        self.collision = False
        self.radius = self.base_robot_image.get_width() // 2
        self.rect = self.base_robot_image.get_rect(center=self.estimated_pos)
        self.right_motor_speed = 0
        self.left_motor_speed = 0
        self.motor_offset = self.radius * 2
        self.sensors = self.Sensors()
        self.mu = np.zeros(3)
        self.sigma = np.eye(3)

    def run_sensors(self, screen):
        """
        Activates the sensor system.

        Args:
            screen (Surface): The Pygame screen surface to draw sensor beams.
        """
        # self.sensors.run(self, screen)
        pass

    def kalman_filter(self, forward_kinematics, take_snapshot=False):
        """
        Applies the Kalman filter to update the robot's state estimation based on sensor data and motion model.

        Args:
            forward_kinematics (ForwardKinematics): An instance containing the robot's calculated kinematics.
        """
        self.estimated_angle = forward_kinematics.agent_angle
        dt = 1
        omega = (self.left_motor_speed - self.right_motor_speed) / self.motor_offset
        velocity = (self.left_motor_speed + self.right_motor_speed) / 2
        A = np.eye(3)
        B = np.array([[dt * np.cos(self.estimated_angle), 0],
                      [dt * np.sin(self.estimated_angle), 0],
                      [0, dt]])

        u = np.array([velocity, omega]).T

        # Define process and measurement noise characteristics.
        sigma_Rx, sigma_Ry, sigma_Rtheta = 0.3, 0.3, 0.5
        sigma_Qx, sigma_Qy, sigma_Qtheta = 1, 1, 2
        R = np.diag([sigma_Rx**2, sigma_Ry**2, sigma_Rtheta**2])
        Q = np.diag([sigma_Qx**2, sigma_Qy**2, sigma_Qtheta**2])

        # Prediction step.
        mu_prediction = A @ self.mu + B @ u
        sigma_prediction = A @ self.sigma @ A.T + R

        # Measurement update.
        # z = np.array([forward_kinematics.agent_pos.x, forward_kinematics.agent_pos.y, forward_kinematics.agent_angle])
        z, is_located = self.environment.get_observation(forward_kinematics, take_snapshot, mu_prediction[2])

        if is_located:
            #Correction
            C = np.eye(3)
            K = sigma_prediction @ C.T @ np.linalg.inv(C @ sigma_prediction @ C.T + Q)
            self.mu = mu_prediction + K @ (z - C @ mu_prediction)
            self.sigma = (np.eye(3) - K @ C) @ sigma_prediction
        else:
            self.mu = mu_prediction
            self.sigma = sigma_prediction

        # Update estimated state with random noise to simulate real-world uncertainty.
        new_pos = np.random.multivariate_normal(self.mu, self.sigma, 1)
        self.estimated_pos = pygame.math.Vector2(new_pos[0, 0], new_pos[0, 1])
        self.estimated_angle = new_pos[0, 2]