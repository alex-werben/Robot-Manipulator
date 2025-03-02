import gymnasium as gym
from stable_baselines3.common.utils import set_random_seed


def make_env(env_id: str, rank: int, log_dir: str, train_from_scratch: bool = True, seed: int = 0):
	def _init() -> gym.Env:
		env = gym.make(env_id, reward_type="dense")
		# env = Monitor(env, f"{log_dir}/{rank}", override_existing=train_from_scratch)
		# env = Monitor(env)
		return env

	set_random_seed(seed)
	return _init
