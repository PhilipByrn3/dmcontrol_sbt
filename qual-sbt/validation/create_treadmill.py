from dm_control import mjcf
import yaml
from pathlib import Path

class Belt:
    def __init__(self, name:str, belt_pos:list,
                 color:list, incline=0):
        self._mjcf_model = mjcf.RootElement(model=name)
        self._belt = self._mjcf_model.worldbody.add(
            'body', name=f'{name}_belt', pos=belt_pos,
            axisangle=[1, 0, 0, incline]
            )
        self._belt.add(
            'geom', type='box', size=[0.05, 10, 0.1],
            friction=[1.0, 0.1, 0.1], mass=3, rgba=color,
            
        )
        self._belt.add(
            'joint', type='slide', pos=[0, -10, 0], axis=[0, 1, 0],
            range=[-0.001, 0.001]
        )

class AssembleTreadmill:
    def __init__(self):
        self._mjcf_model = mjcf.RootElement(model='sbt')
        self._slow_belt = Belt(name='slow',
                               belt_pos=[0, 0, 0],
                               color=[1, 0, 0, 1],
                               incline=0)
        self._fast_belt = Belt(name='fast',
                               belt_pos=[0.1, 0, 0],
                               color=[0, 0, 1, 1],
                               incline=0)
        self._mjcf_model.attach(self._slow_belt._mjcf_model)
        self._mjcf_model.attach(self._fast_belt._mjcf_model)