from sbtenv.validation.check_reference import REFERENCE, validation_stats


def test_defaults_reproduce_reference_statistics():
    # Exact equality: holds on the pinned dm_control 1.0.22 / mujoco 3.2.2.
    assert validation_stats() == REFERENCE
