import pygame
import numpy as np 
import cv2 as cv
from settings import *
import matplotlib.pyplot as plt
import scipy
from skspatial.objects import Circle

class Environment:

    class Landmark:
        def __init__(self, id, position):
            self.id = id
            self.position = position
            self.flag = False

        def draw_landmark(self, screen, radius):
            pygame.draw.circle(screen, ((0, 0, 255) if not self.flag else (255, 255, 0)), self.position, radius)

    def __init__(self):       
        self.maze_array = self.get_pixel_map("images/fourth_maze.png")  # Get binary pixel map of the maze
        # self.landmarks_positions = self.create_landmarks()
        self.landmarks = self.create_landmarks()
        self.landmark_radius = 5
        self.current_selection = []
        self.last_robot_pos = [0, 0]

    def get_pixel_map(self, image_path, threshold=254):
        """
        Pixel map initialization.

        """
        # Load the image using OpenCV
        image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
        # Apply a binary threshold to distinguish walls from free space
        _, binary_image = cv.threshold(image, threshold, 255, cv.THRESH_BINARY_INV)
        # Convert binary image to match the display dimensions
        binary_image = cv.resize(binary_image, (WIDTH, HEIGHT))
        return binary_image

    def is_wall(self, x, y):
        # Ensure x and y are within the bounds of the maze array
        if 0 <= x < self.maze_array.shape[1] and 0 <= y < self.maze_array.shape[0]:
            return self.maze_array[y][x] != 0
        else:
            # If out of bounds, treat as a wall to prevent errors
            return True

    def detect_collision(self, old_pos, new_pos, take_snapshot=False):
        """
        Checks whether a collision occured during a kinematics step.

        Args:
            old_pos: old position before the kinematics step
            new_pos: new position after the kinematics step 
        """

        circle = np.zeros((HEIGHT, WIDTH), dtype='uint8')
        circle = cv.circle(circle, (int(new_pos[0]), int(new_pos[1])), ROBOT_RADIUS, 255, -1)

        inter = circle & self.maze_array       
        # print(f"Circle: {circle}")
        # print(f": {circle}") 

        coords = np.argwhere(inter != 0)

        # print(self.maze_array)
        if coords.size == 0:
            return new_pos, False

        #check if the motion is outward
        circle_old = np.zeros((HEIGHT, WIDTH), dtype='uint8')
        circle_old = cv.circle(circle_old, (int(old_pos[0]), int(old_pos[1])), ROBOT_RADIUS, 255, -1)
        inter_old = circle_old & self.maze_array

        inter = inter.astype('bool')
        inter_old = inter_old.astype('bool')

        if (np.sum(inter) <= np.sum(inter_old)):
            return new_pos, False

        #if 2 points of contact => full stop
        footprint = np.zeros((HEIGHT, WIDTH), dtype='int')
        footprint[coords[:, 0], coords[:, 1]] = 1
        labeled, num_labels = scipy.ndimage.label(footprint, structure=np.ones((3, 3)))

        if num_labels > 1:
            #print(set(labeled.flatten())) 
            return old_pos, True

        ys = coords[:, 0]
        xs = coords[:, 1]

        rng = (np.max(xs) - np.min(xs), np.max(ys) - np.min(ys))

        if take_snapshot:
            print(rng)

            plt.imshow(self.maze_array ^ circle, cmap='gray')
            plt.show()

        #return if any collision => full stop
        #return old_pos, True

        # vertical collision
        if rng[0] > rng[1]:
            adj_pos = (new_pos[0], old_pos[1])
            return adj_pos, True

        # horizontal collision
        if rng[1] > rng[0]:
            adj_pos = (old_pos[0], new_pos[1])
            #return adj_pos, True
            return adj_pos, True
        
        return new_pos, True
    
    def dist(self, pos, robot_pos):
        dx = (pos[0] - robot_pos[0])**2
        dy = (pos[1] - robot_pos[1])**2
        return np.sqrt(dx + dy)

    def wall_before_landmark(self, robot_pos, landmark_pos, take_snapshot):
        """
        Checks whether there is a wall on the segment between landmark and robot positions 
        (landmarks are not visible to robot through the walls)

        Args:
            robot_pos: robot position
            landmark_pos: landmark position
        """

        line = np.zeros((HEIGHT, WIDTH), dtype='uint8')
        line = cv.line(line, (int(robot_pos[0]), int(robot_pos[1])), (int(landmark_pos[0]), int(landmark_pos[1])), 255, 2)

        circle = np.zeros((HEIGHT, WIDTH), dtype='uint8')
        circle = cv.circle(circle, (int(landmark_pos[0]), int(landmark_pos[1])), ROBOT_RADIUS, 255, -1)

        _, circle = cv.threshold(circle, 127, 255, cv.THRESH_BINARY_INV)

        inter = line & (self.maze_array & circle)
        
        if take_snapshot:
            plt.imshow(self.maze_array ^ line, cmap='gray')
            plt.show()
        
        return np.sum(inter) > 0

    def val_close(self, v1, v2, v3):
        a = np.array([v1[0], v2[0], v3[0]])
        b = np.array([v1[1], v2[1], v3[1]])
        return ((np.max(a) - np.min(a)) < 1) and ((np.max(b) - np.min(b)) < 1)

    def derive_location(self, ldm_ids, robot_pos, robot_angle, __estimated_angle):
        """
        Derives location based on 3 landmarks (and a bearing for the angle). 

        Args:
            ldm_ids: landmarks ids selected for observation
            robot_pos: position of the robot
            __estimated_angle: estimated angle of the robot for bearing measurement 
            (actual angle is hidden here and the robot only uses its assumptions and the bearing)
        """

        circles = []
        for id in ldm_ids:
            ldm_pos = self.landmarks[id].position
            dst = self.dist(ldm_pos, robot_pos)
            circles.append(Circle(ldm_pos, dst))

        pt1 = circles[0].intersect_circle(circles[1])
        pt2 = circles[1].intersect_circle(circles[2])
        pt3 = circles[0].intersect_circle(circles[2])
        
        fnx = 0
        fny = 0

        for i in range(2):
            for j in range(2):
                for k in range(2):
                    if self.val_close(pt1[i], pt2[j], pt3[k]):
                        fnx = pt1[i][0]
                        fny = pt1[i][1]

        #bearing part
        ldm_pos = self.landmarks[ldm_ids[0]].position
        vec = np.subtract(ldm_pos, robot_pos)
        
        if np.linalg.norm(vec) < 1e-16:
            ldm_pos = self.landmarks[ldm_ids[1]].position
            vec = ldm_pos - robot_pos
        vec_n = vec / (np.linalg.norm(vec) + 1e-16)
        ldm_angle = np.arctan2(vec_n[1], vec_n[0])
        bearing_observation = __estimated_angle - ldm_angle
        estimated_angle = bearing_observation + ldm_angle 

        final_observation = np.array([fnx, fny, estimated_angle])
        return final_observation

    def get_observation(self, forward_kinematics, take_snapshot, __estimated_angle):
        """
        Retrieves observation for the Kalman filter if possible

        """

        landmark_srt = []
        self.current_selection = []

        robot_pos = [forward_kinematics.agent_pos.x, forward_kinematics.agent_pos.y]
        robot_angle = forward_kinematics.agent_angle
        
        for i in range(len(self.landmarks)): 
            landmark = self.landmarks[i]
            self.landmarks[i].flag = False
            landmark_srt.append((self.dist(landmark.position, robot_pos), landmark.id))

        landmark_srt = sorted(landmark_srt)
        
        selected_ids = []
        for path_len, ldm_id in landmark_srt:
            if path_len < 300 and not self.wall_before_landmark(robot_pos, self.landmarks[ldm_id].position, take_snapshot):
                selected_ids.append(ldm_id)
                self.landmarks[ldm_id].flag = True

        if len(selected_ids) < 3:
            return np.empty(0), False

        cur_ldm_ids = selected_ids[:3]
        self.current_selection = cur_ldm_ids
        self.last_robot_pos = robot_pos
        return self.derive_location(cur_ldm_ids, robot_pos, robot_angle, __estimated_angle), True

    def draw_landmarks(self, screen):
        """
        Drawing called on each frame

        """

        for landmark in self.landmarks:
            landmark.draw_landmark(screen, self.landmark_radius)

        for id in self.current_selection:
            pygame.draw.line(screen, (0, 255, 0), self.last_robot_pos, self.landmarks[id].position)
    
    def create_landmarks(self):
        """
        Initializes landmarks

        """

        pos = [[145,150],
        [145,310],
        [280,155],
        [385,279],
        [178,227],
        [161,412],
        [258,420],
        [53,653],
        [500,653],
        [409,546],
        [627,426],
        [626,305],
        [412,49],
        [744,52],
        [1226,51],
        [1226,279],
        [581,157],
        [1006,185],
        [1010,430],
        [774,651],
        [770,549]]

        #ids = [f"L{x}" for x in range(1, len(pos)+1)]
        ids = np.arange(len(pos))
        landmarks = []

        for i, j in zip(ids, pos):
            landmarks.append(self.Landmark(id=i, position=j))
            
        return landmarks
    