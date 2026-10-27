"""
Rubber contact x fast-belt offset ablation (writeup Section II-G, Table I, Fig. 3).

Simulates every digitized experimental belt speed difference (BSD) for all
eight combinations of rubber placement (none, fast spoke only, slow spoke
only, both) and fast-belt z-offset (none, 4.9 mm). Each configuration is
compared point by point with the experimental velocities at those same BSDs,
giving R2, RMSE and MAE; the figure plots exactly those simulated points.
Invalid trials (fewer than min_rotations) are left out of that
configuration's statistics and figure.

It also times every env.step() call and reports, per configuration, the
average timestep time (the mean of each episode's average) and the total
number of timesteps run.
"""
import csv
import time

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


def run_trial(env, bsd):
    """
    Run one episode at belt speed difference bsd. Returns the average velocity
    (NaN if invalid), the number of timesteps, and the average time per timestep (s).
    """
    obs, info = env.reset(options={'slow_belt_speed': 0.0, 'fast_belt_speed': bsd})
    steps, elapsed = 0, 0.0
    while True:
        action = env.action_space.sample()
        start = time.perf_counter()
        obs, reward, terminated, truncated, info = env.step(action)
        elapsed += time.perf_counter() - start
        steps += 1
        if terminated or truncated:
            break
    velocity = info['avg_velocity'] if info['valid'] else np.nan
    return velocity, steps, elapsed / steps


def paired_stats(sim_v, exp_v):
    """R2, RMSE and MAE between simulated and experimental velocities at the same BSDs, skipping invalid trials."""
    ok = ~np.isnan(sim_v)
    return {'R2': stats.r2(sim_v[ok], exp_v[ok]),
            'RMSE': stats.rmse(sim_v[ok], exp_v[ok]),
            'MAE': stats.mae(sim_v[ok], exp_v[ok]),
            'n': int(ok.sum())}


exp_bsd, exp_v = load_experimental()

# Eight sweeps
results = {}
for rubber, (fast_rubber, slow_rubber) in RUBBER.items():
    for offset, fast_zoffset in OFFSETS.items():
        env = gym.make('sbtenv/RimlessWheel-v0',
                       wheel_kwargs={'fast_rubber': fast_rubber, 'slow_rubber': slow_rubber},
                       treadmill_kwargs={'fast_zoffset': fast_zoffset})
        v, steps, step_time = map(np.array, zip(*(run_trial(env, bsd) for bsd in exp_bsd)))
        results[rubber, offset] = {'v': v, **paired_stats(v, exp_v),
                                   'avg_timestep_s': float(step_time.mean()),
                                   'total_timesteps': int(steps.sum())}
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
                     'points_used': r['n'], 'invalid_trials': invalid,
                     'avg_timestep_s': r['avg_timestep_s'], 'total_timesteps': r['total_timesteps']})

# Timing
print('\n-= Ablation Timing =-')
for offset in OFFSETS:
    for rubber in RUBBER:
        r = results[rubber, offset]
        print(f'{offset.capitalize()}, {rubber.lower()} configuration had an average timestep time of '
              f'{r["avg_timestep_s"]:.3e} seconds for {r["total_timesteps"]} total timesteps')

# Figure (Fig. 3): one row per rubber placement, one column per offset
plt.rcParams['font.family'] = ['Arial', 'DejaVu Sans']
fig, axes = plt.subplots(len(RUBBER), len(OFFSETS), figsize=(9, 12), sharex=True, sharey=True)
for i, rubber in enumerate(RUBBER):
    for j, offset in enumerate(OFFSETS):
        ax = axes[i, j]
        r = results[rubber, offset]
        ax.scatter(exp_bsd, r['v'], s=12, color='blue', label='RW-env', zorder=3)
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
