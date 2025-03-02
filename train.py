import multiprocessing
from stable_baselines3 import A2C, PPO
import gymnasium as gym
from callback import SaveOnBestTrainingRewardCallback
from src.utils import make_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

from env import KochReachObject

def make_env():
    def _init():
        env = Monitor(KochReachObject("DIRECT"), f"./log/")
        
        return env
    return _init
save_callback = SaveOnBestTrainingRewardCallback(
    check_freq=1000,
    log_dir="./log",
    model_dir="./model",
    verbose=1
)
num_cpu = multiprocessing.cpu_count()
# env = SubprocVecEnv([make_env() for i in range(num_cpu)])
env = Monitor(KochReachObject("DIRECT"), f"./log/")
model = PPO("MlpPolicy", env, verbose=0)
model.learn(
    total_timesteps=1_000_000,
    progress_bar=True,
    callback=save_callback,
)

model.save("model.zip")
