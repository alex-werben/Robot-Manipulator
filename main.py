import pybullet as p
import pybullet_data
import time
from src.simulation import Simulation
from src.robot import Robot
from src.dynamixel import Dynamixel
import numpy as np
import time


from stable_baselines3 import A2C, PPO
from stable_baselines3.common.monitor import Monitor
from env import KochReachObject

def _angle_to_pos(joint_angles):
    positions = (((joint_angles + np.pi) / (2 * np.pi)) * 4096).astype(int)
    positions[1] = -(positions[1] - 4096)
    positions[4] = -(positions[4] - 4096) + 2048
    return positions


def main():
    env = KochReachObject("GUI")
    model = A2C.load("model/best_model_cpp.zip", env=env)
    follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name="/dev/tty.usbmodem58FA0959341").instantiate()
    follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])

    new_joint_positions = _angle_to_pos(np.array(env._joint_positions))
    follower.set_goal_pos(new_joint_positions)

    obs, _ = env.reset()

    for i in range(10000):
        time.sleep(1. / 30.)
        action, _ = model.predict(obs, deterministic=True)
        try:
            obs, reward, done, _, _ = env.step(action)
        except p.error:
            follower._disable_torque()
            p.disconnect()  
            return      

        new_joint_positions = _angle_to_pos(np.array(env._joint_positions))
        follower.set_goal_pos(new_joint_positions)

        if done:
            obs, _ = env.reset()
            new_joint_positions = _angle_to_pos(np.array(env._joint_positions))
            follower.set_goal_pos(new_joint_positions)

    # follower._disable_torque()
    # p.disconnect()


main()
# follower._disable_torque()
# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# joint1 = p.addUserDebugParameter("joint1", -3.14, 3.14, 0)
# joint2 = p.addUserDebugParameter("joint2", -3.14, 3.14, 0)
# joint3 = p.addUserDebugParameter("joint3", -3.14, 3.14, 0)
# joint4 = p.addUserDebugParameter("joint4", -3.14, 3.14, 0)
# joint5 = p.addUserDebugParameter("joint5", -3.14, 3.14, 3.14)
# joint6 = p.addUserDebugParameter("joint6", -3.14, 3.14, 0)

# x_param = p.addUserDebugParameter("x", -1, 1, 0)
# y_param = p.addUserDebugParameter("y", -1, 1, 0)
# z_param = p.addUserDebugParameter("z", -1, 1, 0)
# joint_num = p.getNumJoints(robot)
# ee_index = joint_num - 1

# joint_positions = [0, 0, 0, 0, 3.14, 0]
# index = 0
# for j in range(p.getNumJoints(robot)):
#     p.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
#     info = p.getJointInfo(robot, j)
#     joint_type = info[2]
#     if joint_type == p.JOINT_PRISMATIC or joint_type == p.JOINT_REVOLUTE:
#         p.resetJointState(robot, j, joint_positions[index])
#         index = index + 1
        
# ee_position = p.getLinkState(robot, ee_index)[0]
