import platform
import sys

import pytest

from sbtenv.validation.check_reference import REFERENCE, validation_stats

# The reference numbers were produced on an Apple-silicon Mac and reproduce
# bit for bit there. MuJoCo builds for other platforms round slightly
# differently, and the walking dynamics amplify that, so the check is skipped
# elsewhere (CI still prints each platform's statistics).
REFERENCE_PLATFORM = sys.platform == 'darwin' and platform.machine() == 'arm64'


@pytest.mark.skipif(not REFERENCE_PLATFORM, reason='exact values hold only on Apple-silicon macOS')
def test_reproduces_reference_exactly():
    # Exact equality: holds on the pinned dm_control 1.0.22 / mujoco 3.2.2.
    assert validation_stats() == REFERENCE
