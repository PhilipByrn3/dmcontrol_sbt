import gymnasium as gym
import numpy as np
import sbtenv                                   # registers 'sbtenv/RimlessWheel-v0'
from sbtenv.utils.data import load_experimental
from sbtenv.utils import stats

exp_bsd_array, exp_v = load_experimental()

env = gym.make('sbtenv/RimlessWheel-v0')
v = np.empty_like(exp_bsd_array)
for i, bsd in enumerate(exp_bsd_array):
    obs, info = env.reset(options={'slow_belt_speed': 0.0, 'fast_belt_speed': bsd})
    while True:
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            break
    v[i] = info['avg_velocity'] if info['valid'] else np.nan

print('-= Stats =-')
print('RMSE', stats.rmse(v, exp_v))
print('Bias', stats.bias(v, exp_v))
print('R2', stats.r2(v, exp_v))
print('Pearson Correlation: ', stats.pearson_r(v, exp_v))
