import platform
import sys

import pytest

from sbtenv.validation.check_reference import REFERENCE, validation_stats

# The reference numbers were produced on an Apple-silicon Mac. MuJoCo builds
# for other platforms round slightly differently, and the walking dynamics
# amplify that, so elsewhere the statistics are only checked to a tolerance.
REFERENCE_PLATFORM = sys.platform == 'darwin' and platform.machine() == 'arm64'

# About 2-3x the largest change seen when the initial push is perturbed by
# 1e-12 to 1e-10 (RMSE/bias ~0.0011, R2 ~0.043, Pearson ~0.028).
TOLERANCE = {'RMSE': 0.003, 'Bias': 0.003, 'R2': 0.10, 'Pearson': 0.06}


@pytest.fixture(scope='module')
def stats():
    return validation_stats()


@pytest.mark.skipif(not REFERENCE_PLATFORM, reason='exact values hold only on Apple-silicon macOS')
def test_reproduces_reference_exactly(stats):
    # Exact equality: holds on the pinned dm_control 1.0.22 / mujoco 3.2.2.
    assert stats == REFERENCE


def test_reproduces_reference_within_tolerance(stats):
    for name, expected in REFERENCE.items():
        assert stats[name] == pytest.approx(expected, abs=TOLERANCE[name]), name
