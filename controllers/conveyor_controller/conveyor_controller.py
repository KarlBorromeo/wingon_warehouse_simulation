from controller import Supervisor, Motor
import math

def get_box_proto(x, y, z, add_def=True):
    if add_def:
        return f"""
        DEF BOX_1 CardboardBox {{
            translation {x} {y} {z}
            size 0.31 0.37 0.24
            mass 1
        }}
        """
    
    return f"""
    CardboardBox {{
        translation {x} {y} {z}
        size 0.31 0.37 0.24
        mass 1
    }}
    """

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

belt_motor = robot.getDevice("belt_motor")
belt_motor.setPosition(float('inf'))

box_spawner = robot.getFromDef("box_spawner")
box_spawner_tf = box_spawner.getField("translation")

box = None
spawned = False
pallet = None

x, y, z = box_spawner_tf.getSFVec3f()
while robot.step(timestep) != -1:
    belt_motor.setVelocity(0.3)

    if not pallet:
        pallet = robot.getFromDef("PALLET")

    if not spawned:
        box_proto = get_box_proto(x, y, z)

        robot.getRoot().getField("children").importMFNodeFromString(-1, box_proto)
        spawned = True
        continue

    if box is None:
        box = robot.getFromDef("BOX_1")
        continue

    pos = box.getPosition()
    distance_travelled = math.hypot(pos[0] - x, pos[2] - z)
    print(f"Distance travelled by box: {distance_travelled:.3f}")

    if distance_travelled >= 2.7:
        print("Box has reached the end of the conveyor.")

        pallet_box_slot1 = pallet.getField("boxSlot1")
        pallet_box_slot2 = pallet.getField("boxSlot2")

        if pallet_box_slot1.getCount() == 0:
            pallet_box_slot1.importMFNodeFromString(0, get_box_proto(0.0, 0.0, 0.0, add_def=False))
        elif pallet_box_slot2.getCount() == 0:
            pallet_box_slot2.importMFNodeFromString(0, get_box_proto(0.0, 0.0, 0.0, add_def=False))
        else:
            box.remove()

        box = None
        spawned = False


        
