from dm_control import mjcf
import yaml
from pathlib import Path

CONFIG_PATH = Path(__file__).with_name('config.yaml')  # same folder as create_wheel.py
with CONFIG_PATH.open('r') as f:
    config_settings = yaml.safe_load(f)
axle_length = config_settings['axle_length']
slow_side = config_settings['slow_side']
fast_side = config_settings['fast_side']
twoalpha = config_settings['twoalpha']
twobeta = config_settings['twobeta']
relative_spoke_angle = config_settings['relative_spoke_angle']
component_mass = config_settings['component_mass']
slow_side_rubber_attachment = config_settings['slow_side_rubber_attachment']
fast_side_rubber_attachment = config_settings['fast_side_rubber_attachment']
wheel_z_offset = config_settings ['wheel_z_offset']



class Spoke:
    def __init__(self, name:str, axle_length:float, side:int,
                 offset_angle:float, relative_spoke_angle:float, 
                 component_mass:float, rubber_spoke_ending:bool,
                 color:list):
        self._mjcf_model = mjcf.RootElement(model=name)
        
        
             
class Axle:
    def __init__(self, name:str, axle_length:float,
                 component_mass:float):
        self._mjcf_model = mjcf.RootElement(model=name)
        
        self._axle = self._mjcf_model.worldbody.add(
            'body', name='axle', pos=[0, 0, wheel_z_offset]
            )
        self._axle_z_joint = self._axle.add(
            'joint', name='axle_z_joint', type='slide', 
            axis=[0, 0, 1]
        )
        self._axle_y_joint = self._axle.add(
            'joint', name='axle_y_joint', type='slide',
            axis=[0, 1, 0]
        )
        self._axle_hinge_joint = self._axle.add(
            'joint', name='axle_hinge_joint', type='hinge',
            axis=[1, 0, 0]
        )
        self._axle.add(
            'geom', name='axle', type='cylinder', 
            size=[0.01, axle_length], rgba=[0.7, 0.7, 0.7, 1],
            euler=[0, 90, 0], mass=component_mass 
        )
        
        self._spokes = self._axle.worldbody.add(
            'body', name=f'{name}_spokes', pos=[side*axle_length, 0, wheel_z_offset],
            axisangle=[1, 0, 0, offset_angle]
        )
        while 360-relative_spoke_angle >= 0:
            self._spoke = self._spokes.add(
                'body', 
                axisangle=[1, 0, 0, relative_spoke_angle]
            )
            self._spoke.add(
                'geom', type='capsule', size=[0.01], 
                fromto=[0, 0, 0,  0, 0, 0.254], mass=component_mass,
                rgba=color
            )
            self._spoke_addition = self._spokes.add(
                'body',
                axisangle=[1, 0, 0, relative_spoke_angle]
            )
            self._spoke_addition.add(
                'geom', type='box', pos=[0, 0, 0.13],
                size=[0.005, 0.07, 0.07], mass=component_mass,
                rgba=color
            )
            if rubber_spoke_ending:
                self._rubber_spoke_ending = self._spoke.add(
                    'body'
                )
                self._rubber_spoke_ending.add(
                    'geom', type='sphere', size=[0.015], pos=[0, 0, 0.254],
                    rgba=[0.1, 0.1, 0.1, 1], friction=[1, 0.1, 0.1],
                    solimp=[0.8, 0.8, 0.01], solref=[0.02, 1]
                )
                self._rubber_spoke_ending.add(
                    'joint', type='slide', axis=[0, 0, 1], pos=[0, 0, 0.254],
                    range=[0, 0.015], stiffness=1000, damping=10
                )
            relative_spoke_angle += 40                
class AssembleWheel:
    def __init__(self):
        self._mjcf_model = mjcf.RootElement(model='aw')
        self._axle = Axle(name='axle', 
                          axle_length=axle_length, 
                          component_mass=component_mass)
        self._mjcf_model.attach(self._axle._mjcf_model)
        
        
'''self._slow_spokes = Spoke(name='slow',
                                  axle_length=axle_length,
                                  side=slow_side,
                                  offset_angle=twoalpha,
                                  relative_spoke_angle=relative_spoke_angle,
                                  component_mass=component_mass,
                                  rubber_spoke_ending=slow_side_rubber_attachment,
                                  color=[0.7, 0, 0, 1])
        self._fast_spokes = Spoke(name='fast',
                                  axle_length=axle_length,
                                  side=fast_side,
                                  offset_angle=twobeta,
                                  relative_spoke_angle=relative_spoke_angle,
                                  component_mass=component_mass,
                                  rubber_spoke_ending=fast_side_rubber_attachment,
                                  color=[0, 0, 0.7, 1])'''