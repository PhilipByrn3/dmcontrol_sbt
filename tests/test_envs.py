import math

import numpy as np
import pytest

from sbtenv.envs.sbt_env import SBTenv
from sbtenv.envs.sbwm_env import SBWMenv
from sbtenv.utils.sbwm_simulation import SBWMSimulation


def test_treadmill_only_env_builds():
    env = SBTenv()
    assert env.physics.model.nbody == 3          # world + two belts
    assert env.physics.model.nsensor == 0


def test_wheel_env_builds_with_overrides():
    default = SBWMenv().physics.model
    no_fast_rubber = SBWMenv(wheel_kwargs={'fast_rubber': False}).physics.model
    assert no_fast_rubber.ngeom < default.ngeom

    raised = SBWMenv(treadmill_kwargs={'fast_zoffset': 0.005}).physics.model
    assert raised.body('slow_belt_body').pos[2] == 0
    assert raised.body('fast_belt_body').pos[2] == pytest.approx(0.005)


def test_trial_settings_forward_to_task():
    env = SBWMenv(max_rotations=3)
    assert env.task.max_rotations == 3
    env.settle_time = 0.1
    assert env.task.settle_time == 0.1


def test_episode_is_deterministic():
    sim = SBWMSimulation(verbose=False)
    assert sim.run_episode(0.0, 0.5) == sim.run_episode(0.0, 0.5)


def test_stalled_trial_is_invalid():
    sim = SBWMSimulation(verbose=False)
    assert math.isnan(sim.run_episode(0.0, 0.0))     # belts stopped: wheel stalls


def test_max_rotations_ends_the_episode_early():
    sim = SBWMSimulation(verbose=False, max_rotations=1)
    sim.run_episode(0.0, 0.5)
    result = sim.task.episode_result(sim.physics)
    assert result['steps'] < sim.time_limit / sim.timestep
    assert result['rotations'] >= 1


@pytest.mark.parametrize('start, end, step, n', [(0.15, 1.15, 0.05, 21), (0.15, 1.10, 0.05, 20),
                                                 (0.1, 1.0, 0.1, 10), (0.2, 0.2, 0.05, 1)])
def test_sweep_range_includes_both_ends(monkeypatch, start, end, step, n):
    sim = SBWMSimulation(verbose=False)
    monkeypatch.setattr(sim, 'run_episode', lambda slow, fast: fast)
    bsd, v = sim.sweep_episode(0.0, start, end, step)
    assert len(bsd) == n
    assert bsd[0] == start and bsd[-1] == pytest.approx(end)
    np.testing.assert_array_equal(v, bsd)


def test_sweep_needs_a_range_or_array():
    with pytest.raises(ValueError):
        SBWMSimulation(verbose=False).sweep_episode(0.0, 0.15)
