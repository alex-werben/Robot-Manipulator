from .robot import Robot
from .dynamixel import Dynamixel
import numpy as np
import time
import mujoco.viewer
import mujoco
from interface import SimulatedRobot
import threading

# follower_dynamixel = Dynamixel.Config(baudrate=1_000_000, device_name="/dev/tty.usbmodem58FA0959341").instantiate()
# follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])
# follower.tmp_disable_torque()
# follower._enable_torque()

path = "simulation/mujoco/scene.xml"

m = mujoco.MjModel.from_xml_path(path)
d = mujoco.MjData(m)

r = SimulatedRobot(m, d)

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():


        step_start = time.time()

        # sim_robot_position = r.read_position()
        joint_position = r._pos2pwm(sim_robot_position) 
        x = r.d.ctrl[r.m.actuator("x").id]
        ee_pos = r.read_ee_pos("gripper_static_link")
        ee_pos[0] += x
        # print(ee_pos)
        joint_position = r.inverse_kinematics(ee_pos, "gripper_static_link")
        # joint_position[2] += 0.01
        r.d.qpos[7:] = joint_position
        print(d.joint("joint6").xanchor)
        # print(r.d.qpos[7:])
        # print(r.d.ctrl)
        # r.set_target_pos(new_ee_pos)
        mujoco.mj_step(m, d)
        viewer.sync()

        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
