from dm_control import mjcf

def create_sbt_model(sbt_fast_spoke_rubber, sbt_slow_spoke_rubber):
    sbt_model = mjcf.from_path('sbtnew.xml')
    world = sbt_model.worldbody

    twoalpha=15.5
    twobeta=4.5
    genangle = 40
    axle_len = 0.038

    slow_belt = world.add('body', childclass='belt', name='slow_belt')
    slow_belt.add('geom', name='slow_belt_geom', pos=[-0.051, 0, 0], rgba=[1, 0, 0, 1])
    slow_belt.add('joint', name="slow_belt_conveyor")

    fast_belt = world.add('body', childclass='belt', name='fast_belt')
    fast_belt.add('geom', name='fast_belt_geom', pos=[0.051, 0, 0], rgba=[0, 0, 1, 1])
    fast_belt.add('joint', name="fast_belt_conveyor")

    axle_body = world.add('body', name='rimlesswheel', pos='0 0 0.38')
    axle_body.add('geom', name="axle", type="cylinder", size=[0.01, axle_len], rgba="0.7 0 0.7 1", euler="0 90 0", mass="0.095405")
    
    axle_body.add('joint', name="axleZAxis", type="slide", axis="0 0 1")
    axle_body.add('joint', name="axleYAxis", type="slide", axis="0 1 0")
    axle_body.add('joint', name="axleHinge", type="hinge", axis="1 0 0")

    slow_spoke_bodies = axle_body.add('body', name="slow_spoke_bodies", pos=[-1 * axle_len, 0, 0], axisangle=[1, 0, 0, twoalpha] )
    fast_spoke_bodies = axle_body.add('body', name="fast_spoke_bodies", pos=[axle_len, 0, 0], axisangle=[1, 0, 0, twobeta])

    while 360 - genangle >= 0:
        slow_spoke_body = slow_spoke_bodies.add('body', childclass="slow_spoke", axisangle=[1,0,0,genangle])
        slow_spoke_body.add('geom', rgba="0.7 0 0 1")
        slow_spoke_addition = slow_spoke_bodies.add('body', childclass="slow_spoke_addition", axisangle=[1,0,0,genangle])
        slow_spoke_addition.add('geom', rgba="0.7 0.3 0.3 1")
        if sbt_slow_spoke_rubber == True:
            slow_spoke_rubber = slow_spoke_body.add('body', childclass="rubber_pad")
            slow_spoke_rubber.add('geom')

        fast_spoke_body = fast_spoke_bodies.add('body', childclass="fast_spoke", axisangle=[1,0,0,genangle])
        fast_spoke_body.add('geom', rgba="0 0 0.7 1")
        fast_spoke_addition = fast_spoke_bodies.add('body', childclass="fast_spoke_addition", axisangle=[1, 0, 0, genangle])
        fast_spoke_addition.add('geom', rgba="0.3 0.3 0.7 1")
        if sbt_fast_spoke_rubber == True:
            fast_spoke_rubber = fast_spoke_body.add('body', childclass="rubber_pad")
            fast_spoke_rubber.add('geom')

        genangle+=40

    sensor = sbt_model.sensor
    sensor.add('framepos', name='axlepos', objtype="xbody", objname="rimlesswheel")
    
    return sbt_model