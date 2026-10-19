from SBTenv.utils.sbwm_simulation import SBWMSimulation
from SBTenv.utils.stats import Stats
import numpy as np

sim = SBWMSimulation()
exp_bsd_array = np.array([0.170, 0.178, 0.190, 0.199, 0.220, 0.249, 0.268, 0.300, 0.319, 0.350,
              0.369, 0.399, 0.419, 0.449, 0.470, 0.500, 0.519, 0.571, 0.550, 0.601,
              0.620, 0.649, 0.670, 0.701, 0.721, 0.751, 0.772, 0.802, 0.821, 0.851,
              0.872, 0.899, 0.921, 1.001])
exp_v = np.array([0.110, 0.112, 0.123, 0.127, 0.146, 0.159, 0.171, 0.188, 0.188, 0.193,
              0.195, 0.197, 0.200, 0.203, 0.201, 0.202, 0.202, 0.196, 0.204, 0.207,
              0.202, 0.205, 0.198, 0.201, 0.195, 0.197, 0.188, 0.189, 0.187, 0.188,
              0.179, 0.175, 0.174, 0.159])
v = sim.sweep_episode(0, 0.15, 1.15, 0.05, custom_bsd_array=exp_bsd_array)

st = Stats()
print('-= Stats =-')
print('RMSE', st.rmse(v, exp_v))
print('Bias', st.bias(v, exp_v))
print('R2', st.r2(v, exp_v))
print('Pearson Correlation: ', st.pearson_r(v, exp_v))

