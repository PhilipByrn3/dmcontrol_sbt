import gymnasium as gym
import numpy as np
import SBTenv                                   # registers 'SBTenv/RimlessWheel-v0'
from SBTenv.utils.stats import Stats

exp_bsd_array = np.array([0.170, 0.178, 0.190, 0.199, 0.220, 0.249, 0.268, 0.300, 0.319, 0.350,
              0.369, 0.399, 0.419, 0.449, 0.470, 0.500, 0.519, 0.571, 0.550, 0.601,
              0.620, 0.649, 0.670, 0.701, 0.721, 0.751, 0.772, 0.802, 0.821, 0.851,
              0.872, 0.899, 0.921, 1.001])
exp_v = np.array([0.110, 0.112, 0.123, 0.127, 0.146, 0.159, 0.171, 0.188, 0.188, 0.193,
              0.195, 0.197, 0.200, 0.203, 0.201, 0.202, 0.202, 0.196, 0.204, 0.207,
              0.202, 0.205, 0.198, 0.201, 0.195, 0.197, 0.188, 0.189, 0.187, 0.188,
              0.179, 0.175, 0.174, 0.159])

env = gym.make('SBTenv/RimlessWheel-v0')
v = np.empty_like(exp_bsd_array)
for i, bsd in enumerate(exp_bsd_array):
    obs, info = env.reset(options={'slow_belt_speed': 0.0, 'fast_belt_speed': bsd})
    while True:
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            break
    v[i] = info['avg_velocity']

st = Stats()
print('-= Stats =-')
print('RMSE', st.rmse(v, exp_v))
print('Bias', st.bias(v, exp_v))
print('R2', st.r2(v, exp_v))
print('Pearson Correlation: ', st.pearson_r(v, exp_v))

