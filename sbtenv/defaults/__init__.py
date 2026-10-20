"""
Default parameters: the validated configuration used for the research
results (Butterfield et al. 2022 comparison).

Rubber tips on both spoke sides, no fast-belt offset, 0.01 s timestep, RK4.
Validated on dm_control 1.0.22 / mujoco 3.2.2; other MuJoCo versions give
slightly different velocities. validation/check_reference.py checks that
these defaults still reproduce the reference statistics.

    defaults/treadmill.py   SIMULATION, TREADMILL
    defaults/wheel.py       WHEEL, TRIAL

Every class takes these as its defaults; pass keyword arguments to override.
"""
from sbtenv.defaults.treadmill import SIMULATION, TREADMILL
from sbtenv.defaults.wheel import WHEEL, TRIAL
