from dm_control import mjcf
import os

class Treadmill:
    """
    Top-level treadmill assembly. Loads shared materials/skybox assets,
    sets up scene lighting, and composes slow and fast Belt objects.

    Belts are static (jointless) boxes. Belt speed is applied through the
    geom `surfacevel` attribute, which gives contacts on the belt a tangential
    surface velocity without moving the geom. Use set_belt_speeds() to change
    it at runtime. Requires MuJoCo >= 3.14 / dm_control >= 1.0.47 (older
    versions reject the attribute).

    Speed convention: belt speeds are treadmill speeds, positive = the belt
    surface moves backward (-Y), opposite the wheel's walking direction (+Y),
    as on a real treadmill and in Butterfield et al. (2022). A belt moving
    with the wheel (+Y) carries it like a sled and no gait occurs.

    Class constants for geom name lookups:
        Treadmill.SLOW_GEOM, Treadmill.FAST_GEOM

    :param worldbody: MuJoCo worldbody to attach the treadmill to.
    :param xpos: X offset of each belt from center. Slow belt is mirrored to -xpos. Default: 0.051.
    :param belt_size: Box half-extents [x, y, z] of the belt geom. Default: [0.05, 100, 0.1].
    :param zoffset: Vertical offset of both belt bodies. Default: 0.
    :param fast_zoffset: Extra vertical offset of the fast belt only. Default: 0.
        (Butterfield et al. used 4.9 mm in simulation to match their robot.)
    :param belt_friction: MuJoCo friction params [slide, spin, roll]. Default: [1.15, 0.1, 0.1].
    :param belt_mass: Mass of each belt geom in kg. Default: 3.0.
    :param slow_speed: Initial slow belt speed in m/s (positive = backward). Default: 0.
    :param fast_speed: Initial fast belt speed in m/s (positive = backward). Default: 0.
    """
    SLOW_GEOM = 'slow_belt_geom'
    FAST_GEOM = 'fast_belt_geom'
    VALID_KEYS = {
        'xpos',
        'belt_size',
        'zoffset',
        'fast_zoffset',
        'belt_friction',
        'belt_mass',
        'slow_speed',
        'fast_speed'
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

        slow_speed = kwargs.pop('slow_speed', 0.0)
        fast_speed = kwargs.pop('fast_speed', 0.0)
        fast_zoffset = kwargs.pop('fast_zoffset', 0.0)
        zoffset = kwargs.pop('zoffset', 0)
        self.slow_belt = Belt(worldbody, name='slow_belt', color=[1,0,0,1],
                              speed=slow_speed, zoffset=zoffset, **kwargs)
        self.fast_belt = Belt(worldbody, name='fast_belt', color=[0,0,1,1],
                              speed=fast_speed, zoffset=zoffset + fast_zoffset,
                              **kwargs)

    @classmethod
    def set_belt_speeds(cls, physics, slow_speed: float, fast_speed: float):
        """
        Set both belt speeds (m/s, positive = backward along -Y) on a live Physics.

        Replaces writing physics.named.data.qvel for the old conveyor joints.
        Unlike qvel, surfacevel is model state: it persists across env.reset()
        and physics.step(), so it only needs to be set when it changes.
        """
        # Compiled to False when every surfacevel in the XML is zero; without
        # this flag MuJoCo ignores surfacevel values written at runtime.
        physics.model.flg_surfacevel = 1
        sv = physics.named.model.geom_surfacevel
        sv[cls.SLOW_GEOM, 1] = -slow_speed
        sv[cls.FAST_GEOM, 1] = -fast_speed


class Belt:
    """
    One treadmill belt: a static box geom whose surface moves via surfacevel.

    There is no joint: the body is welded to the world and the belt "runs"
    because contacts on the geom see a surface velocity of [0, -speed, 0]
    (geom frame, which is the world frame since the geom is unrotated), i.e.
    a positive speed moves the surface backward like a treadmill.

    Geom name constant: '{name}_geom' (e.g. 'fast_belt_geom').

    :param worldbody: MuJoCo worldbody to attach the belt to.
    :param name: 'slow_belt' or 'fast_belt'. Slow belt is placed at -xpos.
    :param color: RGBA list.
    :param speed: Initial belt speed in m/s (positive = backward). Default: 0.
    :param xpos: X offset from center. Default: 0.051.
    :param belt_size: Box half-extents [x, y, z]. Default: [0.05, 100, 0.1].
    :param zoffset: Vertical offset of the belt body. Default: 0.
    :param belt_friction: MuJoCo friction params [slide, spin, roll]. Default: [1.15, 0.1, 0.1].
    :param belt_mass: Mass of the belt geom in kg. Default: 3.0.
    """
    def __init__(self, worldbody, name, color, speed=0.0, **kwargs):

        xpos = kwargs.get('xpos', 0.051)
        belt_size = kwargs.get('belt_size', [0.05, 100, 0.1])
        zoffset = kwargs.get('zoffset', 0)
        belt_friction = kwargs.get('belt_friction', [1.15, 0.1, 0.1])
        belt_mass = kwargs.get('belt_mass', 3.0)

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
            material='grid',
            surfacevel=[0, -speed, 0]
        )
        self.geom_name = f'{name}_geom'
