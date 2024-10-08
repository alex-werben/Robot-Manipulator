from typing import Any, Dict, Callable

import numpy as np

from gym_envs.envs.core import Task
from gym_envs.pybullet import PyBullet
from gym_envs.utils import distance


class Grasp(Task):
    def __init__(
        self,
        sim: PyBullet,
        reward_type: str = "sparse",
        robot: Callable = None,
        check_collision: Callable = None,
        distance_threshold: float = 0.05,
        goal_xy_range: float = 0.1,
        goal_z_range: float = 0.,
        obj_xy_range: float = 0.1,
    ) -> None:
        super().__init__(sim)
        self.initial_height = 0.0
        self.robot = robot
        self.check_collision = check_collision
        self.reward_type = reward_type
        self.distance_threshold = distance_threshold
        self.object_size = 0.04
        self.goal_range_low = np.array([-goal_xy_range / 2, -goal_xy_range / 2, 0])
        self.goal_range_high = np.array([goal_xy_range / 2, goal_xy_range / 2, goal_z_range])
        self.obj_range_low = np.array([-obj_xy_range / 2, -obj_xy_range / 2, 0])
        self.obj_range_high = np.array([obj_xy_range / 2, obj_xy_range / 2, 0])
        with self.sim.no_rendering():
            self._create_scene()

    def _create_scene(self) -> None:
        """Create the scene."""
        self.sim.create_plane(z_offset=-0.4)
        self.sim.create_table(length=1.1, width=0.7, height=0.4, x_offset=-0.3)
        self.sim.create_box(
            body_name="object",
            half_extents=np.ones(3) * self.object_size / 2,
            mass=1.0,
            position=np.array([0.0, -0.2, self.object_size / 2]),
            rgba_color=np.array([0.1, 0.9, 0.1, 1.0]),
        )
        self.sim.set_lateral_friction("object", -1, 10.0)
        self.sim.set_rolling_friction("object", -1, 0.01)
        self.sim.create_box(
            body_name="target",
            half_extents=np.ones(3) * self.object_size / 2,
            mass=0.0,
            ghost=True,
            position=np.array([0.0, 0.2, 0.1]),
            rgba_color=np.array([0.1, 0.9, 0.1, 0.3]),
        )

    def get_obs(self) -> np.ndarray:
        # position, rotation of the object
        object_position = self.sim.get_base_position("object")
        object_rotation = self.sim.get_base_rotation("object")
        object_velocity = self.sim.get_base_velocity("object")
        object_angular_velocity = self.sim.get_base_angular_velocity("object")
        observation = np.concatenate([object_position, object_rotation, object_velocity, object_angular_velocity])
        return observation

    def get_achieved_goal(self) -> np.ndarray:
        """Returns achieved goal. [0-2] - pos_obj, [3-5] - pos_tcp"""
        pos_obj = np.array(self.sim.get_base_position("object"))
        achieved_goal = np.array(pos_obj)
        return achieved_goal

    def get_desired_goal(self) -> np.ndarray:
        """Return the current goal."""
        if self.goal is None:
            raise RuntimeError("No goal yet, call reset() first")
        else:
            desired_goal = np.array(self.goal.copy())
            return desired_goal

    def reset(self) -> None:
        self.goal = self._sample_goal()
        object_position = self._sample_object()
        self.sim.set_base_pose("target", self.goal, np.array([0.0, 0.0, 0.0, 1.0]))
        self.sim.set_base_pose("object", object_position, np.array([0.0, 0.0, 0.0, 1.0]))
        self.initial_height = self.sim.get_base_position("object")[2]

    def _sample_goal(self) -> np.ndarray:
        """Sample a goal."""
        goal = np.array([0.0, 0.2, 0.15 + self.object_size / 2])  # z offset for the cube center
        noise = self.np_random.uniform(self.goal_range_low, self.goal_range_high)
        if self.np_random.random() < 0.3:
            noise[2] = 0.0
        goal += noise
        return goal

    def _sample_object(self) -> np.ndarray:
        """Randomize start position of object."""
        object_position = np.array([0.0, -0.2, self.object_size / 2])
        noise = self.np_random.uniform(self.obj_range_low, self.obj_range_high)
        object_position += noise
        return object_position

    def is_success(self, achieved_goal: np.ndarray, desired_goal: np.ndarray) -> np.ndarray:
        d = distance(achieved_goal, desired_goal)
        return np.array(d < self.distance_threshold, dtype=bool)

        # d = self.height_diff(achieved_goal)
        # return np.array(d > 0.1, dtype=bool)

    def compute_reward(self, achieved_goal, desired_goal, info: Dict[str, Any]) -> np.ndarray:
        # achieved_goal = np.array(achieved_goal)
        try:
            pos_tcp = np.array([dd["pos_tcp"] for dd in info])
            gripper_action = np.array([dd["grasp"] for dd in info]).astype(bool)
            collisions = np.array([dd["collisions"] for dd in info])
        except:
            pos_tcp = info["pos_tcp"]
            gripper_action = info["grasp"]
            collisions = np.array(info["collisions"]).astype(bool)

        # distance between tcp and object
        reach_reward = distance(pos_tcp, achieved_goal)

        # check if gripper in position for grasp
        position_for_grasp = self.check_position_for_grasp(pos_tcp, achieved_goal)

        # check if caught
        grasp_reward = (position_for_grasp & collisions).astype(int)

        # lift reward and penalty
        # lift_diff = self.height_diff(achieved_goal)

        # penalty
        # penalty_for_overlifting = self.height_penalty(achieved_goal)

        # penalty if there's collision and object not in gripper
        penalty_for_pushing = (collisions & (~ position_for_grasp)).astype(int)

        # reward for grasp
        # holding_reward = (collisions & position_for_grasp).astype(int)

        # distance between object and target
        obj_to_target = distance(achieved_goal, desired_goal)

        # total_reward = - reach_reward + 10 * grasp_reward \
        #                - 2 * penalty_for_pushing + \
        #                - 10 * obj_to_target

        total_reward = - reach_reward + position_for_grasp.astype(int) \
                       + 5 * grasp_reward - 10 * obj_to_target - 2 * penalty_for_pushing

        return total_reward.astype(np.float32)

    def height_penalty(self, pos_obj):
        pos_obj = pos_obj.T
        diff = pos_obj[2] - self.initial_height
        return (diff > 0.15).astype(int)

    def height_diff(self, pos_obj):
        pos_obj = pos_obj.T
        diff = np.clip(pos_obj[2] - self.initial_height, a_min=0, a_max=0.15)
        return np.array(diff)

    def check_position_for_grasp(self, pos_tcp, pos_obj):
        pos_tcp = pos_tcp.T
        pos_obj = pos_obj.T
        position_for_grasp = np.array((np.abs(pos_tcp[0] - pos_obj[0]) < 0.02)
                                      & (np.abs(pos_tcp[1] - pos_obj[1]) < 0.02)
                                      & (np.abs(pos_tcp[2] - pos_obj[2]) < 0.01)).astype(bool)

        return position_for_grasp
