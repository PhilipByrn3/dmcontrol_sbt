from dm_control import mjcf
import os

from SBTenv.defaults.treadmill import TREADMILL

class Treadmill:
    """
    Top-level treadmill assembly. Loads shared materials/skybox assets,
    sets up scene lighting, and composes slow and fast Belt objects.

    Class constants for joint name lookups:
        Treadmill.SLOW_JOINT, Treadmill.FAST_JOINT

    :param worldbody: MuJoCo worldbody to attach the treadmill to.
    :param xpos: X offset of each belt from center. Slow belt is mirrored to -xpos. Default: 0.051.
    :param belt_size: Box half-extents [x, y, z] of the belt geom. Default: [0.05, 100, 0.1].
    :param zoffset: Vertical offset of both belt bodies. Default: 0.
    :param fast_zoffset: Extra vertical offset of the fast belt only, in meters. Default: 0.
    :param belt_friction: MuJoCo friction params [slide, spin, roll]. Default: [1.15, 0.1, 0.1].
    :param belt_mass: Mass of each belt geom in kg. Default: 3.0.

    Defaults come from SBTenv.defaults.treadmill.TREADMILL (the validated configuration).
    """
    SLOW_JOINT = 'slow_belt_conveyor'
    FAST_JOINT = 'fast_belt_conveyor'
    VALID_KEYS = {
        'xpos',
        'belt_size',
        'zoffset',
        'fast_zoffset',
        'belt_friction',
        'belt_mass'
    }
    def __init__(self, worldbody, **kwargs):
        invalid = set(kwargs) - self.VALID_KEYS
        if invalid:
            raise ValueError(f'Unknown params: {invalid}')
        
        root = worldbody.root
        common_dir = os.path.join(os.path.dirname(__file__), 'common')
        materials = mjcf.from_path(os.path.join(common_dir, 'materials.xml'))
        
        root.worldbody.add('light',
            name='top_light',
            pos=[0, 0, 3],
            dir=[0, 0, -1],
            diffuse=[1, 1, 1],
            specular=[0.7, 0.7, 0.7],
            directional=True,
            castshadow=False,
        )

        root.visual.headlight.diffuse = [0.8, 0.8, 0.8]
        root.visual.headlight.ambient = [0.5, 0.5, 0.5]
        root.visual.headlight.specular = [0.3, 0.3, 0.3]

        for texture in materials.asset.find_all('texture'):
            root.asset.add('texture', **texture.get_attributes())

        for material in materials.asset.find_all('material'):
            root.asset.add('material', **material.get_attributes())
            
        skybox = mjcf.from_path(os.path.join(common_dir, 'skybox.xml'))

        for texture in skybox.asset.find_all('texture'):
            root.asset.add('texture', **texture.get_attributes())
        
        fast_zoffset = kwargs.pop('fast_zoffset', TREADMILL['fast_zoffset'])
        self.slow_belt = Belt(worldbody, name='slow_belt', color=[1,0,0,1], **kwargs)
        if fast_zoffset:
            kwargs['zoffset'] = kwargs.get('zoffset', TREADMILL['zoffset']) + fast_zoffset
        self.fast_belt = Belt(worldbody, name='fast_belt', color=[0,0,1,1], **kwargs)


class Belt:
    """
    One treadmill belt: a box geom on a Y-axis slide joint named
    '{name}_conveyor'. SBTTask writes the belt speed to this joint's
    velocity before every physics step. The slow belt is mirrored to -xpos.

    :param worldbody: MuJoCo worldbody to attach the belt to.
    :param name: 'slow_belt' or 'fast_belt'.
    :param color: RGBA list for the belt geom.
    :param xpos: X offset of each belt from center. Slow belt is mirrored to -xpos. Default: 0.051.
    :param belt_size: Box half-extents [x, y, z] of the belt geom. Default: [0.05, 100, 0.1].
    :param zoffset: Vertical offset of the belt body. Default: 0.
    :param belt_friction: MuJoCo friction params [slide, spin, roll]. Default: [1.15, 0.1, 0.1].
    :param belt_mass: Mass of each belt geom in kg. Default: 3.0.
    """
    def __init__(self, worldbody, name, color, **kwargs):
        
        xpos = kwargs.get('xpos', TREADMILL['xpos'])
        belt_size = kwargs.get('belt_size', TREADMILL['belt_size'])
        zoffset = kwargs.get('zoffset', TREADMILL['zoffset'])
        belt_friction = kwargs.get('belt_friction', TREADMILL['belt_friction'])
        belt_mass = kwargs.get('belt_mass', TREADMILL['belt_mass'])
        
        xpos = -xpos if name == 'slow_belt' else xpos
        self._body = worldbody.add(
            'body',
            name=f'{name}_body',
            pos=[xpos, 0, zoffset]
        )
        self._body.add(
            'geom',
            name=f'{name}_geom',
            type='box',
            size=belt_size,
            friction=belt_friction,
            mass=belt_mass,
            rgba=color,
            material='grid'
        )
        self._body.add(
            'joint',
            name=f'{name}_conveyor',
            type='slide',
            pos=[0, -belt_size[1], 0],
            axis=[0, 1, 0],
            range=[-0.001, 0.001]
        )
        self.joint_name = f'{name}_conveyor'

