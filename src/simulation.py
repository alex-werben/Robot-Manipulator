import time
import numpy as np
import pybullet as p
import pybullet_data


class Simulation:
  def __init__(
    self,
    model_path: str = "simulation/model/robot.urdf"
  ) -> None:
    p.connect(p.GUI)
    p.resetSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.setAdditionalSearchPath("/Users/alexander/Developer/Robot-Manipulator/")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(solverResidualThreshold=0)


    self.orientation = p.getQuaternionFromEuler([0, 0, 0])
    self.robot = p.loadURDF(
        fileName=model_path,
        useFixedBase=True,
        globalScaling=0.0025
    )
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    self.table = p.loadURDF(
      "table/table.urdf",
      basePosition=[0, 0, -0.62]
    )
    
    self.block = p.loadURDF(
      fileName="simulation/assets/cylinder.urdf",
      basePosition=[0.1, 0.2, 0.2],
      globalScaling=0.5
    )

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    self.x_param = p.addUserDebugParameter("x", -1, 1, 0)
    self.y_param = p.addUserDebugParameter("y", -1, 1, 0)
    self.z_param = p.addUserDebugParameter("z", -1, 1, 0)
    self.joint_num = p.getNumJoints(self.robot)
    self.ee_index = self.joint_num - 1
    self.x_delta = 0.
    self.y_delta = 0.
    self.z_delta = 0.
    self.a_delta = 0.
    
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
            
    self.ee_position = p.getLinkState(self.robot, self.ee_index)[0]
  
  def step(self):
    p.stepSimulation()
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    
    action = self._handle_keyboard_events()
    
    target_position = np.array(self.ee_position)

    target_position += action[:3]

    joint_positions = p.calculateInverseKinematics(self.robot, self.ee_index, target_position, self.orientation)[:-1]
    
    p.setJointMotorControlArray(self.robot, list(range(0, self.joint_num - 1)), p.POSITION_CONTROL, joint_positions)
    p.setJointMotorControl2(self.robot, self.joint_num - 1, p.POSITION_CONTROL, action[-1])
    new_joint_positions = np.array([joint_info[0] for joint_info in p.getJointStates(self.robot, list(range(0, self.joint_num)))])

    new_joint_positions = self._angle_to_pos(new_joint_positions)

    time.sleep(1. / 240.)
    
    return new_joint_positions
    # follower.set_goal_pos(new_joint_positions)
  
  def _angle_to_pos(self, joint_angles):
    positions = (((joint_angles + np.pi) / (2 * np.pi)) * 4096).astype(int)
    positions[1] = -(positions[1] - 4096)
    positions[4] = -(positions[4] - 4096) + 2048
    return positions
  
  def _handle_keyboard_events(self):
    keys = p.getKeyboardEvents()
    # q = 0
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
      p.disconnect()
    if ord('d') in keys and keys[ord('d')] & p.KEY_WAS_TRIGGERED:
      self.a_delta += 0.05
    if ord('f') in keys and keys[ord('f')] & p.KEY_WAS_TRIGGERED:
      self.a_delta -= 0.05
    if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_TRIGGERED:
      self.x_delta += 0.05
    if ord('n') in keys and keys[ord('n')] & p.KEY_WAS_TRIGGERED:
      self.x_delta -= 0.05
    if ord('b') in keys and keys[ord('b')] & p.KEY_WAS_TRIGGERED:
      self.y_delta += 0.05
    if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
      self.y_delta -= 0.05
    if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED:
      self.z_delta += 0.05
    if ord('z') in keys and keys[ord('z')] & p.KEY_WAS_TRIGGERED:
      self.z_delta -= 0.05
    
    return [self.x_delta, self.y_delta, self.z_delta, self.a_delta]



# while True:
#     time.sleep(1. / 240.)


#     target_position = list(ee_position)

    
#     target_position[0] += x_delta
#     target_position[1] += y_delta
#     target_position[2] += z_delta

#     joint_positions = p.calculateInverseKinematics(robot, ee_index, target_position, orientation)[:-1]
    
#     p.setJointMotorControlArray(robot, list(range(0, joint_num - 1)), p.POSITION_CONTROL, joint_positions)
#     p.setJointMotorControl2(robot, joint_num - 1, p.POSITION_CONTROL, a_delta)
#     new_joint_positions = np.array([joint_info[0] for joint_info in p.getJointStates(robot, list(range(0, joint_num)))])

#     new_joint_positions = angle_to_pos(new_joint_positions)

#     follower.set_goal_pos(new_joint_positions)
        
# follower._disable_torque()

# p.disconnect()
