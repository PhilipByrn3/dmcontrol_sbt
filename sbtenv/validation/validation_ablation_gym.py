"""
Rubber contact x fast-belt offset ablation (writeup Section II-G, Table I, Fig. 3).

Runs the BSD sweep (0.15 to 1.1 m/s in 0.05 m/s steps) for all eight
combinations of rubber placement (none, fast spoke only, slow spoke only,
both) and fast-belt z-offset (none, 4.9 mm). Each configuration is compared
with the digitized experimental data by linearly interpolating its velocity
curve at the experimental BSDs that fall inside the simulated range
(Section II-H), giving R2, RMSE and MAE.
"""
import csv

import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import sbtenv                                   # registers 'sbtenv/RimlessWheel-v0'
from sbtenv.utils.data import load_experimental
from sbtenv.utils import stats
from sbtenv.utils.plotting import save_figure

RUBBER = {                      # row label: (fast_rubber, slow_rubber)
    'No rubber': (False, False),
    'Fast rubber only': (True, False),
    'Slow rubber only': (False, True),
    'Both rubber': (True, True),
}
OFFSETS = {'No offset': 0.0, '4.9 mm offset': 0.0049}     # fast-belt z-offset (m)
SWEEP_BSD = np.arange(0.15, 1.10 + 0.025, 0.05)


def run_trial(env, bsd):
    """Run one episode at belt speed difference bsd; return the average velocity (NaN if invalid)."""
    obs, info = env.reset(options={'slow_belt_speed': 0.0, 'fast_belt_speed': bsd})
    while True:
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            break
    return info['avg_velocity'] if info['valid'] else np.nan


def interpolated_stats(sim_bsd, sim_v, exp_bsd, exp_v):
    """R2, RMSE and MAE of the sim curve interpolated at the experimental BSDs inside its range."""
    ok = ~np.isnan(sim_v)
    sim_bsd, sim_v = sim_bsd[ok], sim_v[ok]
    inside = (exp_bsd >= sim_bsd.min()) & (exp_bsd <= sim_bsd.max())
    predicted = np.interp(exp_bsd[inside], sim_bsd, sim_v)
    return {'R2': stats.r2(predicted, exp_v[inside]),
            'RMSE': stats.rmse(predicted, exp_v[inside]),
            'MAE': stats.mae(predicted, exp_v[inside]),
            'n': int(inside.sum())}


exp_bsd, exp_v = load_experimental()

# Eight sweeps
results = {}
for rubber, (fast_rubber, slow_rubber) in RUBBER.items():
    for offset, fast_zoffset in OFFSETS.items():
        env = gym.make('sbtenv/RimlessWheel-v0',
                       wheel_kwargs={'fast_rubber': fast_rubber, 'slow_rubber': slow_rubber},
                       treadmill_kwargs={'fast_zoffset': fast_zoffset})
        v = np.array([run_trial(env, bsd) for bsd in SWEEP_BSD])
        results[rubber, offset] = {'v': v, **interpolated_stats(SWEEP_BSD, v, exp_bsd, exp_v)}
        print(f'Finished: {rubber.lower()}, {offset.lower()}')

# Stats table (Table I)
print('\n-= Ablation Stats =-')
print(f'{"Configuration":34s} {"R2":>9s} {"RMSE (m/s)":>11s} {"MAE (m/s)":>10s} {"invalid":>8s}')
rows = []
for offset in OFFSETS:
    for rubber in RUBBER:
        r = results[rubber, offset]
        invalid = int(np.isnan(r['v']).sum())
        name = f'{rubber}, {offset.lower()}'
        print(f'{name:34s} {r["R2"]:9.3f} {r["RMSE"]:11.4f} {r["MAE"]:10.4f} {invalid:8d}')
        rows.append({'configuration': name, 'R2': r['R2'], 'RMSE': r['RMSE'], 'MAE': r['MAE'],
                     'experimental_points': r['n'], 'invalid_trials': invalid})

# Figure (Fig. 3): one row per rubber placement, one column per offset
plt.rcParams['font.family'] = ['Arial', 'DejaVu Sans']
fig, axes = plt.subplots(len(RUBBER), len(OFFSETS), figsize=(9, 12), sharex=True, sharey=True)
for i, rubber in enumerate(RUBBER):
    for j, offset in enumerate(OFFSETS):
        ax = axes[i, j]
        r = results[rubber, offset]
        ax.scatter(SWEEP_BSD, r['v'], s=12, color='blue', label='RW-env', zorder=3)
        ax.scatter(exp_bsd, exp_v, s=12, color='red', label='RW-exp', zorder=3)
        ax.text(0.97, 0.95, f'$R^2$ = {r["R2"]:.3f}\nRMSE = {r["RMSE"]:.4f} m/s',
                transform=ax.transAxes, ha='right', va='top', fontsize=8)
        ax.set_ylim(0, 0.40)
        ax.set_yticks(np.arange(0, 0.401, 0.10))
        ax.spines[['top', 'right']].set_visible(False)
        if i == 0:
            ax.set_title(offset, fontsize=12)
        if j == 0:
            ax.set_ylabel(f'{rubber}\nAvg Steady Velocity (m/s)', fontsize=10)
        if i == len(RUBBER) - 1:
            ax.set_xlabel('Belt Speed Difference (m/s)', fontsize=10)

handles, labels = axes[0, 0].get_legend_handles_labels()
legend = fig.legend(handles, labels, loc='upper right', fontsize=11, frameon=True, fancybox=True,
                    markerscale=1.5, bbox_to_anchor=(0.98, 0.985))
legend.get_frame().set_facecolor('#d9d9d9')
legend.get_frame().set_edgecolor('#1f2d3d')
fig.suptitle('Rubber Contact and Fast-Belt Offset Ablation', fontsize=15)
fig.tight_layout(rect=(0, 0.02, 1, 0.97))

out = save_figure(fig, __file__)
csv_path = out.with_suffix('.csv')
with open(csv_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
print(f'\nFigure saved to {out.resolve()}')
print(f'Stats saved to {csv_path.resolve()}')
plt.show()
