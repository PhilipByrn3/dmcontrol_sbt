import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import sbtenv                                   # registers 'sbtenv/RimlessWheel-v0'
from sbtenv.utils.data import load_experimental
from sbtenv.utils import stats
from sbtenv.utils.plotting import save_figure


def run_trial(env, bsd):
    """Run one episode at belt speed difference bsd; return the average velocity (NaN if invalid)."""
    obs, info = env.reset(options={'slow_belt_speed': 0.0, 'fast_belt_speed': bsd})
    while True:
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            break
    return info['avg_velocity'] if info['valid'] else np.nan


exp_bsd_array, exp_v = load_experimental()
env = gym.make('sbtenv/RimlessWheel-v0')

# Simulate at the experimental belt speed differences; the stats and the
# figure both use exactly these points.
v = np.array([run_trial(env, bsd) for bsd in exp_bsd_array])

print('-= Stats =-')
print('RMSE', stats.rmse(v, exp_v))
print('Bias', stats.bias(v, exp_v))
print('R2', stats.r2(v, exp_v))
print('Pearson Correlation: ', stats.pearson_r(v, exp_v))

plt.rcParams['font.family'] = ['Arial', 'DejaVu Sans']
fig, ax = plt.subplots(figsize=(7, 5))
ax.scatter(exp_bsd_array, v, s=22, color='blue', label='RW-env', zorder=3)
ax.scatter(exp_bsd_array, exp_v, s=22, color='red', label='RW-exp', zorder=3)

ax.set_title('Experimental and Simulated Rimless Wheel Walking\nVelocities on a Split-Belt Treadmill',
             fontsize=16, pad=16)
ax.set_xlabel('Belt Speed Difference (m/s)', fontsize=12)
ax.set_ylabel('Avg Steady Velocity (m/s)', fontsize=12)
ax.set_ylim(0, 0.40)
ax.set_yticks(np.arange(0, 0.401, 0.05))
ax.tick_params(labelsize=11)
ax.spines[['top', 'right']].set_visible(False)

legend = ax.legend(loc='lower right', fontsize=13, frameon=True, fancybox=True,
                   borderpad=0.8, handletextpad=0.6, markerscale=1.2)
legend.get_frame().set_facecolor('#d9d9d9')
legend.get_frame().set_edgecolor('#1f2d3d')
legend.get_frame().set_linewidth(1.2)
legend.get_frame().set_alpha(1)

fig.tight_layout(rect=(0, 0.02, 1, 1))
out = save_figure(fig, __file__)
print(f'Figure saved to {out.resolve()}')
plt.show()
