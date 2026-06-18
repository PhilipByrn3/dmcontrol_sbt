from dm_control import mjcf

class Wheel:
    """
    Top-level rimless wheel assembly. Validates config kwargs and
    composes an Axle with slow and fast SpokeSets.

    Class constants for joint/body name lookups:
        Wheel.JOINT_Z, JOINT_Y, JOINT_HINGE, BODY_NAME

    :param worldbody: MuJoCo worldbody to attach the wheel to.
    :param fast_rubber: Enable rubber tips on fast-side spokes. Default: True.
    :param slow_rubber: Enable rubber tips on slow-side spokes. Default: True.
    :param axle_length: Half-length of the axle cylinder in meters. Default: 0.038.
    :param wheel_height: Z height of the axle body above the ground. Default: 0.38.
    :param spoke_length: Length of each spoke capsule in meters. Default: 0.254.
    :param twoalpha: Angular offset of the slow-side spoke cluster in degrees. Default: 15.5.
    :param twobeta: Angular offset of the fast-side spoke cluster in degrees. Default: 4.5.
    :param component_mass: Mass of each individual spoke geom in kg. Default: 0.095405.
    :param angle_step: Angular spacing between spokes in degrees. Default: 40.
    :param rubber_friction: MuJoCo friction params [slide, spin, roll] for rubber tips. Default: [1, 1, 0.1].
    :param rubber_solimp: Solver impedance params for rubber tip contact. Default: [0.8, 0.8, 0.01].
    :param rubber_solref: Solver reference params for rubber tip contact. Default: [0.02, 1].
    :param rubber_size: Radius of the rubber tip sphere in meters. Default: [0.017].
    :param smd_range: Slide joint range [min, max] for the spring-mass-damper. Default: [0, 0.015].
    :param smd_stiffness: Stiffness of the SMD slide joint in N/m. Default: 50.
    :param smd_damping: Damping of the SMD slide joint in Ns/m. Default: 10.
    """
    JOINT_Z     = 'axleZAxis'
    JOINT_Y     = 'axleYAxis'
    JOINT_HINGE = 'axleHinge'
    BODY_NAME   = 'rimlesswheel'
    VALID_KEYS = {
        'fast_rubber', 
        'slow_rubber', 
        'axle_length',
        'wheel_height'
        'spoke_length',
        'twoalpha',
        'twobeta',
        'component_mass'
        'angle_step'
        'rubber_friction'
        'rubber_solimp',
        'rubber_solref',
        'rubber_size',
        'smd_range',
        'smd_stiffness',
        'smd_damping'
    }

    def __init__(self, worldbody, **kwargs):
        invalid = set(kwargs) - self.VALID_KEYS
        if invalid:
            raise ValueError(f'Unknown params: {invalid}')

        self.axle = Axle(worldbody)
        self.slow_spokes = SpokeSet(
            axle_body=self.axle._body, side=-1)
        self.fast_spokes = SpokeSet(
            axle_body=self.axle._body, side=1)


class SpokeSet:
    """
    Builds one side of the wheel's spokes (slow: side=-1, fast: side=1).
    Spokes are evenly distributed at angle_step increments around the axle.
    Optionally adds a rubber tip with a spring-mass-damper (SMD) slide joint.

    :param axle_body: MuJoCo body to attach the spoke cluster to.
    :param side: -1 for slow side, 1 for fast side. Controls xpos sign, color, and offset angle.
    :param fast_rubber: Enable rubber tips on fast-side spokes. Default: True.
    :param slow_rubber: Enable rubber tips on slow-side spokes. Default: True.
    :param axle_length: Half-length of the axle used to offset the cluster in meters. Default: 0.038.
    :param spoke_length: Length of each spoke capsule in meters. Default: 0.254.
    :param twoalpha: Angular offset of the slow-side spoke cluster in degrees. Default: 15.5.
    :param twobeta: Angular offset of the fast-side spoke cluster in degrees. Default: 4.5.
    :param component_mass: Mass of each individual spoke geom in kg. Default: 0.095405.
    :param angle_step: Angular spacing between spokes in degrees. Default: 40.
    :param rubber_friction: MuJoCo friction params [slide, spin, roll] for rubber tips. Default: [1, 1, 0.1].
    :param rubber_solimp: Solver impedance params for rubber tip contact. Default: [0.8, 0.8, 0.01].
    :param rubber_solref: Solver reference params for rubber tip contact. Default: [0.02, 1].
    :param rubber_size: Radius of the rubber tip sphere in meters. Default: [0.017].
    :param smd_range: Slide joint range [min, max] for the spring-mass-damper. Default: [0, 0.015].
    :param smd_stiffness: Stiffness of the SMD slide joint in N/m. Default: 50.
    :param smd_damping: Damping of the SMD slide joint in Ns/m. Default: 10.
    """

    def __init__(self, axle_body, side: int, **kwargs):
        fast_rubber = kwargs.get('fast_rubber', True)
        slow_rubber = kwargs.get('slow_rubber', True)
        axle_length = kwargs.get('axle_length', 0.038)
        spoke_length = kwargs.get('spoke_length', 0.254)
        twoalpha = kwargs.get('twoalpha', 15.5)
        twobeta = kwargs.get('twobeta', 4.5)
        component_mass = kwargs.get('component_mass', 0.095405)
        angle_step = kwargs.get('angle_step', 40)
        rubber_friction = kwargs.get('rubber_friction', [1, 1, 0.1])
        rubber_solimp = kwargs.get('rubber_solimp', [0.8, 0.8, 0.01])
        rubber_solref = kwargs.get('rubber_solref', [0.02, 1])
        rubber_size = kwargs.get('rubber_size', [0.017])
        smd_range = kwargs.get('smd_range', [0, 0.015])
        smd_stiffness = kwargs.get('smd_stiffness', 50)
        smd_damping = kwargs.get('smd_damping', 10)
        
        side_name      = 'slow' if side < 0 else 'fast'
        offset_angle = twoalpha if side_name == 'slow' else twobeta
        color = [0.7, 0, 0, 1] if side_name == 'slow' else [0, 0, 0.7, 1]
        rubber = slow_rubber if side_name == 'slow' else fast_rubber

        self._cluster = axle_body.add(
            'body',
            name=f'{side_name}_spoke_bodies',
            pos=[side * axle_length, 0, 0],
            axisangle=[1, 0, 0, offset_angle]
        )

        angle = angle_step
        while 360 - angle >= 0:
            spoke_body = self._cluster.add('body', axisangle=[1, 0, 0, angle])
            spoke_body.add(
                'geom',
                type='capsule',
                size=[0.01],
                fromto=[0, 0, 0, 0, 0, spoke_length],
                mass=component_mass,
                rgba=color
            )
            spoke_add = self._cluster.add('body', axisangle=[1, 0, 0, angle])
            spoke_add.add(
                'geom',
                type='box',
                pos=[0, 0, 0.13],
                size=[0.005, 0.07, 0.07],
                mass=component_mass,
                rgba=color
            )
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


class Axle:
    """
    Builds the central axle body with a cylinder geom and three joints:
    Z-slide, Y-slide, and X-hinge — giving the wheel full planar mobility.

    :param worldbody: MuJoCo worldbody to attach the axle body to.
    :param axle_length: Half-length of the axle cylinder in meters. Default: 0.038.
    :param wheel_height: Z height of the axle body above the ground. Default: 0.38.
    :param component_mass: Mass of the axle cylinder geom in kg. Default: 0.095405.
    """
    def __init__(self, worldbody, **kwargs):
        
        axle_length = kwargs.get('axle_length', 0.038)
        wheel_height = kwargs.get('wheel_height', 0.38)
        component_mass = kwargs.get('component_mass', 0.095405)
        
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


