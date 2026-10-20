"""
Check that the default (validated) configuration still reproduces the
reference validation statistics exactly.

Run after any change to the models, tasks, envs or defaults. Exact equality
only holds on the pinned versions (dm_control 1.0.22, mujoco 3.2.2).
"""
import sys

from SBTenv.utils.sbwm_simulation import SBWMSimulation
from SBTenv.utils.data import load_experimental
from SBTenv.utils import stats

REFERENCE = {
    'RMSE': 0.01391385495448577,
    'Bias': -0.0038202663253913112,
    'R2': 0.7374408430580734,
    'Pearson': 0.9228722047286865,
}

exp_bsd, exp_v = load_experimental()
_, v = SBWMSimulation(verbose=False).sweep_episode(0, custom_bsd_array=exp_bsd)
result = {
    'RMSE': stats.rmse(v, exp_v),
    'Bias': stats.bias(v, exp_v),
    'R2': stats.r2(v, exp_v),
    'Pearson': stats.pearson_r(v, exp_v),
}

ok = True
for name, expected in REFERENCE.items():
    match = result[name] == expected
    ok &= match
    print(f'{name:8s} {result[name]!r:24s} {"OK" if match else f"MISMATCH (expected {expected!r})"}')
print('Reference reproduced exactly.' if ok else 'Reference NOT reproduced.')
sys.exit(0 if ok else 1)
