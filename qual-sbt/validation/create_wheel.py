"""
create_wheel.py
Builds the rimless wheel MuJoCo model programmatically.

The wheel is assembled by adding bodies directly to the worldbody of the
top-level model, avoiding mjcf name-scoping complications from attach().

Classes:
    SpokeSet      — one set of N evenly-spaced spokes for one side of the wheel
    Axle          — central axle body with 3 DOFs (Z-slide, Y-slide, X-hinge)
    AssembleWheel — complete wheel: axle + slow-side spokes + fast-side spokes

Joint name constants (use these when accessing physics.named.data):
    AssembleWheel.JOINT_Z      'axleZAxis'
    AssembleWheel.JOINT_Y      'axleYAxis'
    AssembleWheel.JOINT_HINGE  'axleHinge'
    AssembleWheel.BODY_NAME    'rimlesswheel'  (also used by the axlepos sensor)
"""

from dm_control import mjcf
import yaml
from pathlib import Path

CONFIG_PATH = Path(__file__).with_name('config.yaml')


def load_config() -> dict:
    with CONFIG_PATH.open('r') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------

class SpokeSet:
    """
    One set of N spokes for one side of the rimless wheel.

    Adds spoke bodies as children of axle_body. Each spoke consists of:
      - a capsule geom (the spoke itself)
      - a box geom at mid-length (spoke addition, for extra inertia and to accurately model Butterfield exp setup)
      - optionally a rubber pad sphere at the tip (spring-mass-damper contact)

    The rubber pad reduces the bouncing problem at high belt speed differences.
    See CLAUDE.md for full design rationale.
    """

    def __init__(self, axle_body, side: int, offset_angle: float,
                 rubber: bool, color: list, config: dict):
        """
        Args:
            axle_body:    mjcf body element for the axle (rimlesswheel body)
            side:         -1 = slow side, +1 = fast side
            offset_angle: angular offset of this spoke cluster around axle X-axis
                          (twoalpha for slow side, twobeta for fast side), degrees
            rubber:       add compliant rubber pad spheres to each spoke tip
            color:        RGBA list for spoke geoms
            config:       simulation config dict
        """
        axle_length    = config['axle_length']
        spoke_length   = config['spoke_length']
        component_mass = config['component_mass']
        angle_step     = config['spoke_angle_step']
        rubber_friction= config['rubber_friction']
        rubber_solimp  = config['rubber_solimp']
        rubber_solref  = config['rubber_solref']
        rubber_size    = config['rubber_size']
        smd_range      = config['smd_range']
        smd_stiffness  = config['smd_stiffness']
        smd_damping    = config['smd_damping']
        side_name      = 'slow' if side < 0 else 'fast'

        # Cluster body: offset along axle X-axis, pre-rotated by offset_angle
        self._cluster = axle_body.add(
            'body',
            name=f'{side_name}_spoke_bodies',
            pos=[side * axle_length, 0, 0],
            axisangle=[1, 0, 0, offset_angle]
        )

        # Add spokes, equally spaced around the cluster
        angle = angle_step
        while 360 - angle >= 0:
            # Primary spoke: capsule geom
            spoke_body = self._cluster.add('body', axisangle=[1, 0, 0, angle])
            spoke_body.add(
                'geom',
                type='capsule',
                size=[0.01],
                fromto=[0, 0, 0, 0, 0, spoke_length],
                mass=component_mass,
                rgba=color
            )

            # Spoke addition: box geom for inertia (mirrors physical device)
            spoke_add = self._cluster.add('body', axisangle=[1, 0, 0, angle])
            spoke_add.add(
                'geom',
                type='box',
                pos=[0, 0, 0.13],
                size=[0.005, 0.07, 0.07],
                mass=component_mass,
                rgba=color
            )

            # Optional rubber pad: compliant sphere + slide joint at spoke tip
            if rubber:
                rubber_body = spoke_body.add('body')
                rubber_body.add(
                    'geom',
                    type='sphere',
                    size=rubber_size,
                    pos=[0, 0, spoke_length],
                    rgba=[0.1, 0.1, 0.1, 1],
                    friction=rubber_friction,
                    solimp=rubber_solimp,
                    solref=rubber_solref
                )
                rubber_body.add(
                    'joint',
                    type='slide',
                    axis=[0, 0, 1],
                    pos=[0, 0, spoke_length],
                    range=smd_range,
                    stiffness=smd_stiffness,
                    damping=smd_damping
                )

            angle += angle_step


# ---------------------------------------------------------------------------

class Axle:
    """
    Central axle body of the rimless wheel.

    Degrees of freedom:
        axleZAxis  — vertical slide (Z); wheel can rise/fall
        axleYAxis  — forward slide (Y); primary locomotion axis
        axleHinge  — rotation around axle axis (X)

    The wheel body is named 'rimlesswheel' so that the axlepos framepos sensor
    (added by SplitBeltSim) can reference it by name.
    """

    def __init__(self, worldbody, config: dict):
        axle_length    = config['axle_length']
        component_mass = config['component_mass']
        wheel_height   = config['wheel_height']

        self._body = worldbody.add(
            'body',
            name='rimlesswheel',
            pos=[0, 0, wheel_height]
        )
        self._body.add(
            'geom',
            name='axle_geom',
            type='cylinder',
            size=[0.01, axle_length],
            rgba=[0.7, 0, 0.7, 1],
            euler=[0, 90, 0],
            mass=component_mass
        )
        self._body.add('joint', name='axleZAxis', type='slide', axis=[0, 0, 1])
        self._body.add('joint', name='axleYAxis', type='slide', axis=[0, 1, 0])
        self._body.add('joint', name='axleHinge',  type='hinge', axis=[1, 0, 0])


# ---------------------------------------------------------------------------

class AssembleWheel:
    """
    Assembles the complete rimless wheel: axle + slow-side + fast-side spokes.

    Adds all geometry directly to worldbody (no mjcf attach()).
    Joint names remain exactly as defined — no name-scoping prefix.

    Class-level constants for joint/body name lookups:

        AssembleWheel.JOINT_Z      = 'axleZAxis'
        AssembleWheel.JOINT_Y      = 'axleYAxis'
        AssembleWheel.JOINT_HINGE  = 'axleHinge'
        AssembleWheel.BODY_NAME    = 'rimlesswheel'

    Usage:
        wheel = AssembleWheel(model.worldbody, config)
        # Joint names are now live in the model; add sensor in parent:
        model.sensor.add('framepos', name='axlepos',
                         objtype='xbody', objname=AssembleWheel.BODY_NAME)
    """

    JOINT_Z     = 'axleZAxis'
    JOINT_Y     = 'axleYAxis'
    JOINT_HINGE = 'axleHinge'
    BODY_NAME   = 'rimlesswheel'

    def __init__(self, worldbody, config: dict):
        self.axle = Axle(worldbody, config)

        self.slow_spokes = SpokeSet(
            axle_body=self.axle._body,
            side=-1,
            offset_angle=config['twoalpha'],
            rubber=config['slow_side_rubber'],
            color=[0.7, 0, 0, 1],
            config=config
        )
        self.fast_spokes = SpokeSet(
            axle_body=self.axle._body,
            side=1,
            offset_angle=config['twobeta'],
            rubber=config['fast_side_rubber'],
            color=[0, 0, 0.7, 1],
            config=config
        )
