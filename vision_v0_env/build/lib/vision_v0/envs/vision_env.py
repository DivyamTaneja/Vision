import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
import pybullet_data
import cv2 as cv
import numpy as np
import random
from os.path import normpath, basename
import time
class VisionEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, car_location=None, ball_location=None, humanoid_location=None, cam_height=None, visual_cam_settings=None):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        p.setRealTimeSimulation(1)

        if car_location is not None:
            self.car_location = car_location
        else:
            self.car_location = [0,0,0.3]
        if ball_location is not None:
            self.ball_location = ball_location
        else:
            self.ball_location = [2,3,0.3]
        if humanoid_location is not None:
            self.humanoid_location = humanoid_location
        else:
            self.humanoid_location = [5,6,0.3]
        if cam_height is not None:
            self.cam_height = cam_height
        else:
            self.cam_height = 0.1
        if visual_cam_settings!=None:
            p.resetDebugVisualizerCamera(visual_cam_settings['cam_dist'], visual_cam_settings['cam_yaw'], visual_cam_settings['cam_pitch'], visual_cam_settings['cam_target_pos'])
    
        self.load_env()

    def load_env(self):
        self.arena = p.loadURDF('urdf/arena/arena_v0.urdf')
        self.car = p.loadURDF('urdf/car/car.urdf', self.car_location, p.getQuaternionFromEuler([0,0,0]))  
        self.ball = p.loadURDF('urdf/ball/ball_red.urdf', self.ball_location, p.getQuaternionFromEuler([0,0,0]))
        self.humnaoid = p.loadURDF('urdf/humanoid/humanoid_red.urdf', self.humanoid_location, p.getQuaternionFromEuler([0,0,0]))   

    def move(self, vels):
        vels = np.array(vels)
        [left_front, right_front, left_back, right_back] = vels.flatten()
        target_vels = [left_front,-right_front,left_back,-right_back]
        p.setJointMotorControlArray(
            bodyIndex = self.car,
            jointIndices = [0,1,2,3],
            controlMode = p.VELOCITY_CONTROL,
            targetVelocities = target_vels)
        
    def open_grip(self):
        p.setJointMotorControl2(self.car,5,p.POSITION_CONTROL,targetPosition = np.pi/2)
        p.setJointMotorControl2(self.car,6,p.POSITION_CONTROL,targetPosition = -np.pi/2)
        time.sleep(1./240.)

    def close_grip(self):
        p.setJointMotorControl2(self.car,5,p.POSITION_CONTROL,targetPosition = 0)
        p.setJointMotorControl2(self.car,6,p.POSITION_CONTROL,targetPosition = 0)
        time.sleep(1./240.)

    def get_image(self):
        orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.car)[1])
        pos = p.getBasePositionAndOrientation(self.car)[0]
        pos = np.add(pos,np.array([0,0,self.cam_height]))
        camera_eye = [pos[0]+0.4*np.cos(orn[2]),pos[1]+0.4*np.sin(orn[2]),pos[2]+1.15*np.cos(orn[0])]
        target_pos = [pos[0]-2*np.cos(orn[2]),pos[1]-2*np.sin(orn[2]),pos[2]+1.15*np.cos(orn[0])]
        view_matrix = p.computeViewMatrix(camera_eye, target_pos, [0,0,1])
        proj_matrix = p.computeProjectionMatrixFOV(60, 1, 0.02, 50)
        images = p.getCameraImage(512, 512, view_matrix, proj_matrix, shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgba_opengl = np.reshape(images[2], (512, 512, 4))
        rgba_opengl = np.uint8(rgba_opengl)
        bgr = cv.cvtColor(rgba_opengl[:,:,0:3], cv.COLOR_BGR2RGB)  
        return bgr     

    def shoot(self):
        p.setJointMotorControl2(self.car, 8, p.POSITION_CONTROL, targetPosition=-1.5, force=50)
        time.sleep(1./3.)
        p.setJointMotorControl2(self.car, 8, p.POSITION_CONTROL, targetPosition=0, force=50)
        time.sleep(1./3.)    