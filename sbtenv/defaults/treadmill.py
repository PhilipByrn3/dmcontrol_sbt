"""
Treadmill defaults: solver/integration settings (SBTenv) and the split-belt
treadmill (Treadmill / Belt), as used for the validated research results:
no fast-belt offset, 0.01 s timestep, RK4.

Validated on dm_control 1.0.22 / mujoco 3.2.2.
"""

# Solver and integration (SBTenv)
SIMULATION = {
    'timestep': 0.01,           # s
    'integrator': 'RK4',
    'solver': 'PGS',
    'cone': 'Elliptic',
    'iterations': 50,
    'gravity': [0, 0, -9.81],   # m/s^2
}

# Split-belt treadmill (Treadmill / Belt)
TREADMILL = {
    'xpos': 0.051,                      # m, belt centre offset from x = 0 (slow belt mirrored)
    'belt_size': [0.05, 100, 0.1],      # m, box half-extents [x, y, z]
    'zoffset': 0,                       # m, height of both belts
    'fast_zoffset': 0.0,                # m, extra height of the fast belt only
    'belt_friction': [1.15, 0.1, 0.1],  # sliding, torsional, rolling friction
    'belt_mass': 3.0,                   # kg
}
