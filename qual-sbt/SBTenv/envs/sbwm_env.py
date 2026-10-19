from SBTenv.models.wheel import Wheel
from SBTenv.envs.sbt_env import SBTenv
 
class SBWMenv(SBTenv):
    def _add_models(self, wheel_kwargs):
        Wheel(self._model.worldbody, **(wheel_kwargs or {}))     
        self._model.sensor.add('framepos',
                                        name='axlepos',
                                        objtype='xbody',
                                        objname=Wheel.BODY_NAME
                                        )
