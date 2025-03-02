import math
import os
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
from typing import List

MODE = p.GUI  # p.GUI or p.DIRECT - with or without rendering
DIM_OBS = 6  # no. of dimensions in observation space
DIM_ACT = 4  # no. of dimensions in action space
MAX_EPISODE_LEN = 500


class Environment(gym.Env):
    def __init__(self):
        self.step_counter = 0
        self.observation = None
        p.connect(MODE)
        self.reset()

        self.action_space = spaces.Box(np.array([-1] * DIM_ACT), np.array([1] * DIM_ACT))
        self.observation_space = spaces.Box(np.array([-1] * DIM_OBS), np.array([1] * DIM_OBS))

    def reset(self, seed=23):
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setAdditionalSearchPath("/Users/alexander/Developer/Robot-Manipulator/")
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(solverResidualThreshold=0)

        self.step_counter = 0        
        self.orientation = p.getQuaternionFromEuler([0., 0., 0.])
        # robot
        self.robot = p.loadURDF(
            "simulation/model/robot.urdf",
            useFixedBase=True,
            basePosition=[0, 0, 0],
            globalScaling=0.0025,
            baseOrientation=self.orientation
        )
        self.joint_num = p.getNumJoints(self.robot)
        self.ee_index = self.joint_num - 1
        
        # TODO: fix here probably (3.14 -> 0), then it can be removed at all
        joint_positions = [0, 0, 0, 0, 3.14, 0]
        index = 0
        for j in range(p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.robot, j)
            joint_type = info[2]
            if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
                p.resetJointState(self.robot, j, joint_positions[index])
                index = index + 1

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.table = p.loadURDF(
            "table/table.urdf",
            basePosition=[0, 0, -0.62]
        )


        self.object = p.loadURDF(
            fileName="simulation/assets/cylinder.urdf",
            basePosition=[0.5, 0.2, 0.2],
            globalScaling=0.5
        )
        
        # Debug print axes
        p.addUserDebugText('X', [1, 0, 0], [0, 0, 0])
        p.addUserDebugText('Y', [0, 1, 0], [0, 0, 0])

        state_robot = np.array(p.getLinkState(self.robot, self.ee_index)[0])
        state_object = self._get_object_position()

        observation = state_robot + state_object

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        return observation

    def step(self, action: List):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)

        # Execute action
        dv = 0.005
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv
        ee_angle = action[3] * dv

        current_position = p.getLinkState(self.robot, self.ee_index)[0]
        new_position = [
            current_position[0] + dx,
            current_position[1] + dy,
            current_position[2] + dz
        ]

        joint_positions = p.calculateInverseKinematics(
            self.robot,
            self.ee_index,
            new_position,
            self.orientation
        ) # TODO: check if it is correct to remove last joint from array
        
        p.setJointMotorControlArray(
            self.robot, 
            list(range(self.joint_num)),
            p.POSITION_CONTROL,
            joint_positions
        )

        p.stepSimulation()
        # time.sleep(1/ 240.)

        state_object = self._get_object_position()
        state_robot = p.getLinkState(self.robot, self.ee_index)[0]
        reward = self._calculate_reward(state_robot, state_object)

        self.step_counter += 1
        done = False
        if self.step_counter > MAX_EPISODE_LEN or reward > -0.5:
            done = True

        print("REWARD: ", reward)

        info = {}

        observation = state_robot + state_object

        return observation, reward, done, info

    def _calculate_reward(self, *args) -> float:
        state_robot = args[0]
        state_object = args[1]

        distance = np.linalg.norm(state_object - state_robot)

        reward = -distance

        return reward

    def _get_object_position(self):
        """
        Get the position of the object.
        TODO: realize it with camera.

        Returns
        -------
        state_object : array_like
            The position of the object as a 3-element array.
        """
        state_object = np.array(p.getBasePositionAndOrientation(self.object)[0])

        return state_object

    def render(self):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7, 0, 0.8],
                                                          distance=.7,
                                                          yaw=90,
                                                          pitch=-70,
                                                          roll=0,
                                                          upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                   aspect=float(960) / 720,
                                                   nearVal=0.1,
                                                   farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                            height=720,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720, 960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        p.disconnect()

env = Environment()
env.reset()
# while (1):
for _ in range(100000):
    # env.render()
    action = env.action_space.sample()
    print(action)
    env.step(action)
    # env.step(1)
env.close()