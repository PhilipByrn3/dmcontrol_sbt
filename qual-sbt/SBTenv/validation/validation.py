from SBTenv.utils.sbwm_simulation import SBWMSimulation
from SBTenv.utils.data import load_experimental
from SBTenv.utils import stats

sim = SBWMSimulation()
exp_bsd_array, exp_v = load_experimental()
bsd, v = sim.sweep_episode(0, custom_bsd_array=exp_bsd_array)

print('-= Stats =-')
print('RMSE', stats.rmse(v, exp_v))
print('Bias', stats.bias(v, exp_v))
print('R2', stats.r2(v, exp_v))
print('Pearson Correlation: ', stats.pearson_r(v, exp_v))
