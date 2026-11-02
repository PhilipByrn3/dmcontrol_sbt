"""
Rimless wheel defaults: the wheel (Wheel / SpokeSet / Axle) and the
validation trial (SBWMenv / SBWMTask), as used for the validated research
results: rubber tips on both spoke sides.

Validated on dm_control 1.0.22 / mujoco 3.2.2.
"""

# Rimless wheel (Wheel / SpokeSet / Axle)
WHEEL = {
    'fast_rubber': True,
    'slow_rubber': True,
    'axle_length': 0.038,       # m, half-length
    'wheel_height': 0.38,       # m, initial axle height
    'spoke_length': 0.254,      # m
    'alpha': 15.5,              # deg, half the fast->slow spoke angle (2*alpha = 31)
    'beta': 4.5,                # deg, half the slow->fast spoke angle (2*beta = 9)
    'wheel_angle': 10.0,        # deg, initial rotation of the whole wheel
    'component_mass': 0.095405, # kg, per spoke / spoke addition / axle
    'rubber_friction': [1, 1, 0.1],
    'rubber_solimp': [0.8, 0.8, 0.01],
    'rubber_solref': [0.02, 1],
    'rubber_size': [0.017],     # m, rubber tip radius
    'rubber_mass': None,        # kg per rubber tip; None = MuJoCo default density (20.6 g)
    'smd_range': [0, 0.015],    # m, spring-mass-damper slide range
    'smd_stiffness': 50,        # N/m
    'smd_damping': 10,          # Ns/m
}

# Validation trial (SBWMenv / SBWMTask)
TRIAL = {
    'time_limit': 20,           # s
    'settle_time': 0.05,        # s, discarded before measuring
    'min_rotations': 1,         # rotations during measurement for a valid trial
    'max_rotations': 12,        # rotations from reset that end the episode
}
