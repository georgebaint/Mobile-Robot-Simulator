# World setting
WIDTH = 1280
HEIGHT = 720
FPS = 120

# Robot settings
ROBOT_START_X = 320
ROBOT_START_Y = 340
ROBOT_SIZE = 0.25
ROBOT_MAX_SPEED = 3
ROBOT_RADIUS = 15

SPEED_DELTA = 0.15 #0.5
ROTATION_SPEED_DELTA = 0.2 #0.6

# Noise parameters

# Standard Deviation of noise of motion model
SIGMA_RX, SIGMA_RY, SIGMA_RZ = 0.2, 0.2, 0.1
# Standard Deviation of noise of sensor model
SIGMA_QX, SIGMA_QY, SIGMA_QZ = 1, 1, 5

LANDMARK_RADIUS = 5

# SIMULATION SETTINGS

VISUALIZE = True

ITER_COUNT = 1600
COUNTER_DROP = 200
MAZE_NUM = 2