from SBTenv.models.treadmill import Treadmill
from SBTenv.models.wheel import Wheel
from dm_control import mujoco, mjcf, viewer
from dm_control.rl.control import Environment
from dm_control.suite.base import Task

class SplitBeltTask(Task):
    def before_step(self, action, physics):
        pass
    def get_observation(self, physics):
        pass
    def get_reward(self, physics):
        pass
    def initialize_episode(self, physics):
        pass

model = mjcf.RootElement(model='sbt_rimless_wheel')
model.compiler.autolimits = True
model.compiler.angle      = 'degree'
model.option.gravity    = [0, 0, -9.81]
Wheel(model.worldbody)
Treadmill(model.worldbody, belt_size=[0.05,10,0.1])
physics = mujoco.Physics.from_xml_string(model.to_xml_string())
env     = Environment(physics=physics, task=SplitBeltTask())
viewer.launch(env)