from dm_control import mjcf
import os

class Treadmill:
    SLOW_JOINT = 'slow_belt_conveyor'
    FAST_JOINT = 'fast_belt_conveyor'
    VALID_KEYS = {
        'xpos',
        'belt_size',
        'zoffset',
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
        
        self.slow_belt = Belt(worldbody, name='slow_belt', color=[1,0,0,1], **kwargs)
        self.fast_belt = Belt(worldbody, name='fast_belt', color=[0,0,1,1], **kwargs)


class Belt:
    def __init__(self, worldbody, name, color, **kwargs):
        
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

