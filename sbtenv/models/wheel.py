from dm_control import mjcf

from sbtenv.defaults.wheel import WHEEL

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
    :param alpha: Half the angle between a fast spoke and the next slow spoke,
        in degrees (Butterfield et al. 2022). Default: 15.5 (2*alpha = 31).
    :param beta: Half the angle between a slow spoke and the next fast spoke,
        in degrees. Default: 4.5 (2*beta = 9). The two spoke sets are offset
        by 2*beta, and the spokes on each side are 2*(alpha + beta) apart,
        which must divide 360.
    :param wheel_angle: Initial rotation of the whole wheel about the axle in
        degrees; the spoke sets sit at wheel_angle +/- beta. Default: 10.
    :param component_mass: Mass of each individual spoke geom in kg. Default: 0.095405.
    :param rubber_friction: MuJoCo friction params [slide, spin, roll] for rubber tips. Default: [1, 1, 0.1].
    :param rubber_solimp: Solver impedance params for rubber tip contact. Default: [0.8, 0.8, 0.01].
    :param rubber_solref: Solver reference params for rubber tip contact. Default: [0.02, 1].
    :param rubber_size: Radius of the rubber tip sphere in meters. Default: [0.017].
    :param rubber_mass: Mass of each rubber tip in kg. Default: None, which
        lets MuJoCo use its default density (1000 kg/m^3, 20.6 g per tip).
    :param smd_range: Slide joint range [min, max] for the spring-mass-damper. Default: [0, 0.015].
    :param smd_stiffness: Stiffness of the SMD slide joint in N/m. Default: 50.
    :param smd_damping: Damping of the SMD slide joint in Ns/m. Default: 10.

    Defaults come from sbtenv.defaults.wheel.WHEEL (the validated configuration).
    """
    JOINT_Z     = 'axleZAxis'
    JOINT_Y     = 'axleYAxis'
    JOINT_HINGE = 'axleHinge'
    BODY_NAME   = 'rimlesswheel'
    VALID_KEYS = {
        'fast_rubber', 
        'slow_rubber', 
        'axle_length',
        'wheel_height',
        'spoke_length',
        'alpha',
        'beta',
        'wheel_angle',
        'component_mass',
        'rubber_friction',
        'rubber_solimp',
        'rubber_solref',
        'rubber_size',
        'rubber_mass',
        'smd_range',
        'smd_stiffness',
        'smd_damping'
    }

    def __init__(self, worldbody, **kwargs):
        invalid = set(kwargs) - self.VALID_KEYS
        if invalid:
            raise ValueError(f'Unknown params: {invalid}')
        spacing = 2 * (kwargs.get('alpha', WHEEL['alpha']) + kwargs.get('beta', WHEEL['beta']))
        if 360 % spacing:
            raise ValueError(f'2*(alpha + beta) = {spacing} deg must divide 360')

        self.axle = Axle(worldbody, **kwargs)
        self.slow_spokes = SpokeSet(
            axle_body=self.axle._body, side=-1, **kwargs)
        self.fast_spokes = SpokeSet(
            axle_body=self.axle._body, side=1, **kwargs)


class SpokeSet:
    """
    Builds one side of the wheel's spokes (slow: side=-1, fast: side=1).
    Spokes are evenly distributed at 2*(alpha + beta) increments around the axle.
    Optionally adds a rubber tip with a spring-mass-damper (SMD) slide joint.

    :param axle_body: MuJoCo body to attach the spoke cluster to.
    :param side: -1 for slow side, 1 for fast side. Controls xpos sign, color, and offset angle.
    :param fast_rubber: Enable rubber tips on fast-side spokes. Default: True.
    :param slow_rubber: Enable rubber tips on slow-side spokes. Default: True.
    :param axle_length: Half-length of the axle used to offset the cluster in meters. Default: 0.038.
    :param spoke_length: Length of each spoke capsule in meters. Default: 0.254.
    :param alpha: Half the fast->slow spoke angle in degrees. Default: 15.5.
    :param beta: Half the slow->fast spoke angle in degrees. Default: 4.5.
    :param wheel_angle: Initial rotation of the whole wheel in degrees. Default: 10.
    :param component_mass: Mass of each individual spoke geom in kg. Default: 0.095405.
    :param rubber_friction: MuJoCo friction params [slide, spin, roll] for rubber tips. Default: [1, 1, 0.1].
    :param rubber_solimp: Solver impedance params for rubber tip contact. Default: [0.8, 0.8, 0.01].
    :param rubber_solref: Solver reference params for rubber tip contact. Default: [0.02, 1].
    :param rubber_size: Radius of the rubber tip sphere in meters. Default: [0.017].
    :param rubber_mass: Mass of each rubber tip in kg. Default: None (MuJoCo default density).
    :param smd_range: Slide joint range [min, max] for the spring-mass-damper. Default: [0, 0.015].
    :param smd_stiffness: Stiffness of the SMD slide joint in N/m. Default: 50.
    :param smd_damping: Damping of the SMD slide joint in Ns/m. Default: 10.
    """

    def __init__(self, axle_body, side: int, **kwargs):
        fast_rubber = kwargs.get('fast_rubber', WHEEL['fast_rubber'])
        slow_rubber = kwargs.get('slow_rubber', WHEEL['slow_rubber'])
        axle_length = kwargs.get('axle_length', WHEEL['axle_length'])
        spoke_length = kwargs.get('spoke_length', WHEEL['spoke_length'])
        alpha = kwargs.get('alpha', WHEEL['alpha'])
        beta = kwargs.get('beta', WHEEL['beta'])
        wheel_angle = kwargs.get('wheel_angle', WHEEL['wheel_angle'])
        component_mass = kwargs.get('component_mass', WHEEL['component_mass'])
        angle_step = 2 * (alpha + beta)
        rubber_friction = kwargs.get('rubber_friction', WHEEL['rubber_friction'])
        rubber_solimp = kwargs.get('rubber_solimp', WHEEL['rubber_solimp'])
        rubber_solref = kwargs.get('rubber_solref', WHEEL['rubber_solref'])
        rubber_size = kwargs.get('rubber_size', WHEEL['rubber_size'])
        rubber_mass = kwargs.get('rubber_mass', WHEEL['rubber_mass'])
        smd_range = kwargs.get('smd_range', WHEEL['smd_range'])
        smd_stiffness = kwargs.get('smd_stiffness', WHEEL['smd_stiffness'])
        smd_damping = kwargs.get('smd_damping', WHEEL['smd_damping'])
        
        side_name      = 'slow' if side < 0 else 'fast'
        offset_angle = wheel_angle + beta if side_name == 'slow' else wheel_angle - beta
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
                    mass=rubber_mass,
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
        
        axle_length = kwargs.get('axle_length', WHEEL['axle_length'])
        wheel_height = kwargs.get('wheel_height', WHEEL['wheel_height'])
        component_mass = kwargs.get('component_mass', WHEEL['component_mass'])
        
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


