"""
Mass and moment of inertia of the rimless wheel about its axle.

    python -m sbtenv.utils.wheel_inertia
    python -m sbtenv.utils.wheel_inertia --rubber-mass 0.027 --component-mass 0.082

The inertia is the axle hinge's diagonal entry of MuJoCo's joint-space mass
matrix: the inertia of everything that turns with the axle, about the axle.
"""
import argparse

import mujoco
import numpy as np

from sbtenv.envs.sbwm_env import SBWMenv
from sbtenv.models.wheel import Wheel


def wheel_inertia(physics):
    """Return (mass in kg, inertia about the axle in kg*m^2) of the wheel in physics."""
    m, d = physics.model.ptr, physics.data.ptr
    mujoco.mj_forward(m, d)

    body = next(i for i in range(m.nbody)
                if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or '').endswith(Wheel.BODY_NAME))
    hinge = next(j for j in range(m.njnt)
                 if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or '').endswith(Wheel.JOINT_HINGE))

    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, M, d.qM)
    dof = m.jnt_dofadr[hinge]
    return float(m.body_subtreemass[body]), float(M[dof, dof])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--rubber-mass', type=float, help='kg per rubber tip (default: MuJoCo density)')
    parser.add_argument('--component-mass', type=float, help='kg per spoke, spoke box and axle')
    parser.add_argument('--no-rubber', action='store_true', help='build the wheel without rubber tips')
    args = parser.parse_args()

    wheel_kwargs = {}
    if args.rubber_mass is not None:
        wheel_kwargs['rubber_mass'] = args.rubber_mass
    if args.component_mass is not None:
        wheel_kwargs['component_mass'] = args.component_mass
    if args.no_rubber:
        wheel_kwargs.update(fast_rubber=False, slow_rubber=False)

    mass, inertia = wheel_inertia(SBWMenv(wheel_kwargs=wheel_kwargs).physics)
    print(f'Mass:                 {mass:.4f} kg')
    print(f'Inertia about axle:   {inertia:.5f} kg*m^2')
    print(f'Radius of gyration:   {np.sqrt(inertia / mass):.4f} m')
