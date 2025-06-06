from dm_control import mjcf

def create_sbt_model():
    sbt_model = mjcf.from_path('sbtnew.xml')
    world = sbt_model.worldbody

    twoalpha=15.5
    twobeta=4.5
    genangle = 40

    axle_body = world.add('body', name='rimlesswheel', pos='0 0 0.37')
    axle_body.add('geom', name="axle", type="cylinder", size="0.01 0.07", rgba="0.7 0 0.7 1", euler="0 90 0", mass="0.095405")
    
    axle_body.add('joint', name="axleZAxis", type="slide", axis="0 0 1")
    axle_body.add('joint', name="axleYAxis", type="slide", axis="0 1 0")
    axle_body.add('joint', name="axleHinge", type="hinge", axis="1 0 0")

    slow_spoke_bodies = axle_body.add('body', name="slow_spoke_bodies", pos="-1 0 0", axisangle=[1, 0, 0, twoalpha] )
    fast_spoke_bodies = axle_body.add('body', name="fast_spoke_bodies", pos="1 0 0", axisangle=[1, 0, 0, twobeta])

    while 360 - genangle >= 0:
        slow_spoke_body = slow_spoke_bodies.add('body', childclass="slow_spoke", pos="1 0 0", axisangle=[1,0,0,genangle])
        slow_spoke_body.add('geom', rgba="0.7 0 0 1")

        spoke_slow_addition = slow_spoke_bodies.add('body', childclass="slow_spoke_addition", axisangle=[1,0,0,genangle])
        spoke_slow_addition.add('geom', rgba="0.7 0.3 0.3 1")

        fast_spoke_body = fast_spoke_bodies.add('body', childclass="fast_spoke", pos="-1 0 0", axisangle=[1,0,0,genangle])
        fast_spoke_body.add('geom', rgba="0 0 0.7 1")

        spoke_fast_addition = axle_body.add('body', childclass="fast_spoke_addition", axisangle=[1, 0, 0, genangle])
        spoke_fast_addition.add('geom', rgba="0.3 0.3 0.7 1")

        genangle+=40

    sensor = sbt_model.sensor
    sensor.add('touch', name='slow_touch', site='slow_belt_force_site')
    sensor.add('touch', name='fast_touch', site='fast_belt_force_site')
    sensor.add('framepos', name='axlepos', objtype="xbody", objname="rimlesswheel")
    
    return sbt_model