"""
create_treadmill.py
Builds the split-belt treadmill MuJoCo model programmatically.

Belt velocity is not motor-driven. Instead, SplitBeltSim sets
physics.named.data.qvel for each belt's conveyor joint directly on every
timestep, kinematically imposing the desired speed. The belt joint has a
near-zero range to prevent free sliding while allowing this approach.

Classes:
    Belt              — one kinematically constrained conveyor belt
    AssembleTreadmill — slow belt (red) + fast belt (blue) assembly

Joint name constants (use these when accessing physics.named.data):
    AssembleTreadmill.SLOW_JOINT  'slow_belt_conveyor'
    AssembleTreadmill.FAST_JOINT  'fast_belt_conveyor'
"""

from dm_control import mjcf
import yaml
from pathlib import Path

CONFIG_PATH = Path(__file__).with_name('config.yaml')


def load_config() -> dict:
    with CONFIG_PATH.open('r') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------

class Belt:
    """
    One treadmill belt: a kinematically constrained box geom.

    The belt body is placed at (xpos, 0, zoffset) in the world frame.
    The conveyor joint is a Y-axis slide with a ±0.001 m range, effectively
    pinning the belt in place while allowing SplitBeltSim to overwrite qvel.
    """

    def __init__(self, worldbody, name: str, xpos: float, zoffset: float,
                 color: list, config: dict):
        """
        Args:
            worldbody: mjcf worldbody element
            name:      unique belt identifier, e.g. 'slow_belt' or 'fast_belt'
            xpos:      x-position of belt centre (separates belts along axle axis)
            zoffset:   z-offset; fast belt can be raised slightly to reduce bouncing
            color:     RGBA list
            config:    simulation config dict
        """
        belt_friction = config['belt_friction']
        belt_mass     = config['belt_mass']
        half_w        = config['belt_half_width']
        half_l        = config['belt_half_length']
        half_h        = config['belt_half_height']

        self._body = worldbody.add(
            'body',
            name=f'{name}_body',
            pos=[xpos, 0, zoffset]
        )
        self._body.add(
            'geom',
            name=f'{name}_geom',
            type='box',
            size=[half_w, half_l, half_h],
            friction=belt_friction,
            mass=belt_mass,
            rgba=color
        )
        self._body.add(
            'joint',
            name=f'{name}_conveyor',
            type='slide',
            pos=[0, -half_l, 0],
            axis=[0, 1, 0],
            range=[-0.001, 0.001]
        )

        # Expose joint name for physics access
        self.joint_name = f'{name}_conveyor'


# ---------------------------------------------------------------------------

class AssembleTreadmill:
    """
    Assembles the split-belt treadmill: stationary slow belt + driven fast belt.

    Adds both belts directly to worldbody (no mjcf attach()).

    Class-level constants for joint name lookups:

        AssembleTreadmill.SLOW_JOINT = 'slow_belt_conveyor'
        AssembleTreadmill.FAST_JOINT = 'fast_belt_conveyor'

    Usage:
        treadmill = AssembleTreadmill(model.worldbody, config)
        # physics.named.data.qvel[AssembleTreadmill.FAST_JOINT] = belt_diff
    """

    SLOW_JOINT = 'slow_belt_conveyor'
    FAST_JOINT = 'fast_belt_conveyor'

    def __init__(self, worldbody, config: dict):
        fast_z = config.get('fast_belt_zoffset', 0.0)

        self.slow_belt = Belt(
            worldbody=worldbody,
            name='slow_belt',
            xpos=config['slow_belt_xpos'],
            zoffset=0.0,
            color=[1, 0, 0, 1],
            config=config
        )
        self.fast_belt = Belt(
            worldbody=worldbody,
            name='fast_belt',
            xpos=config['fast_belt_xpos'],
            zoffset=fast_z,
            color=[0, 0, 1, 1],
            config=config
        )
