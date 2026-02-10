from controller import Supervisor, Motor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

belt_motor: Motor = robot.getDevice("belt_motor")
belt_motor.setPosition(float('inf'))
belt_motor.setVelocity(0.0)

# Get box spawner position
box_spawner = robot.getFromDef("box_spawner")
box_spawner_tf = box_spawner.getField("translation")

# Timing
spawn_interval = 5.0  # seconds
last_spawn_time = 0.0
sim_time = 0.0

while robot.step(timestep) != -1:
    belt_motor.setVelocity(0.3)

    sim_time += timestep / 1000.0

    if sim_time - last_spawn_time >= spawn_interval:
        last_spawn_time = sim_time

        x, y, z = box_spawner_tf.getSFVec3f()
        # z += 0.02  # small lift to avoid collision with belt

        box_proto = f"""
        CardboardBox {{
          translation {x} {y} {z}
          rotation -0.3016120512105832 -0.9349314530822059 -0.18690465109819893 2.593254445485388e-06
          size 0.31 0.37 0.24
          mass 1
        }}
        """

        robot.getRoot().getField("children").importMFNodeFromString(
            -1, box_proto
        )
