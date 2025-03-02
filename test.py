from stable_baselines3 import A2C, PPO
import time
from stable_baselines3.common.monitor import Monitor

from env import KochReachObject
env = KochReachObject("GUI")

model = A2C.load("model/best_model_cpp.zip", env=env)

obs, info = env.reset()
for i in range(10000):
    time.sleep(1. / 120.)
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, _, info = env.step(action)
    if done:
        obs, info = env.reset()
