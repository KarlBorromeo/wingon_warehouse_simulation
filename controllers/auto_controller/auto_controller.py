from controller import Robot, Motor, Supervisor, Field, PositionSensor
import math
from dataclasses import dataclass

@dataclass(frozen=True)
class LinearMoveAction:
    distance: float
    speed: float = -2.0
    tol: float = 0.01

@dataclass(frozen=True)
class TurnAction:
    degrees: float
    turn_radius: float = 1.0

@dataclass(frozen=True)
class LiftAction:
    height: float
    speed: float = 0.2

@dataclass(frozen=True)
class SetPalletTransportedAction:
    admin: Supervisor = None

@dataclass(frozen=True)
class WaitAction:
    duration: float

# Constants
speed = -4.0
max_steer_angle = 1.0 
min_forklift_height = 0.001
max_forklift_height = 2.0
wheel_base = 0.83
track_width = 0.63

#PLANS
PLANS = {
    "forklift_1": [
        LinearMoveAction(distance=2.0, speed=speed),
        TurnAction(degrees=-90.0, turn_radius=1.5),
        LinearMoveAction(distance=6.0, speed=speed),
        TurnAction(degrees=90.0, turn_radius=1.0),
        LinearMoveAction(distance=13.5, speed=speed),
        TurnAction(degrees=-85.0, turn_radius=0.5),
        LinearMoveAction(distance=1.5, speed=speed),
        LiftAction(height=2.0, speed=0.2),
        SetPalletTransportedAction(),
        LinearMoveAction(distance=3.0, speed=-speed),
        TurnAction(degrees=180.0, turn_radius=1.0),
        LinearMoveAction(distance=6.0, speed=speed),
        TurnAction(degrees=88.0, turn_radius=1.0),
        LinearMoveAction(distance=4.2, speed=speed),
        LiftAction(height=1.85, speed=0.2),
        LinearMoveAction(distance=3.1, speed=-speed),
        LiftAction(height=min_forklift_height, speed=0.2),
        TurnAction(degrees=85.0, turn_radius=0.5),
        LinearMoveAction(distance=10.4, speed=speed),
        LiftAction(height=1.3, speed=0.2),
        SetPalletTransportedAction(),
        LinearMoveAction(distance=3.0, speed=-speed),
        TurnAction(degrees=180.0, turn_radius=1.0),
        LinearMoveAction(distance=21.15, speed=speed),
        TurnAction(degrees=90.0, turn_radius=1.0),
        TurnAction(degrees=177.0, turn_radius=1.0),
        LinearMoveAction(distance=4.1, speed=speed),
        LiftAction(height=1.13, speed=0.2),
        LinearMoveAction(distance=3.1, speed=-speed),
        LiftAction(height=min_forklift_height, speed=0.2),
    ],

    "forklift_2": [
        WaitAction(duration=12.0),
        LinearMoveAction(distance=2.0, speed=speed),
        TurnAction(degrees=-90.0, turn_radius=0.5),
        LinearMoveAction(distance=2.0, speed=speed),
        TurnAction(degrees=85.0, turn_radius=0.5),
        LinearMoveAction(distance=13.5, speed=speed),
        WaitAction(duration=22.0),
        LinearMoveAction(distance=2.7, speed=speed),
        TurnAction(degrees=-80.0, turn_radius=0.5),
        LinearMoveAction(distance=2.8, speed=speed),
        LiftAction(height=2.0, speed=0.2),
        SetPalletTransportedAction(),
        LinearMoveAction(distance=3.0, speed=-speed),
        TurnAction(degrees=180.0, turn_radius=1.0),
        LinearMoveAction(distance=27.0, speed=speed),
        TurnAction(degrees=90.0, turn_radius=1.0),
        LinearMoveAction(distance=10.0, speed=speed),
        TurnAction(degrees=176.0, turn_radius=1.0),
        LinearMoveAction(distance=2.7, speed=speed),
        LiftAction(height=1.85, speed=0.2),
        LinearMoveAction(distance=3.0, speed=-speed),
        LiftAction(height=min_forklift_height, speed=0.2),
        TurnAction(degrees=-90.0, turn_radius=1.0),
        LinearMoveAction(distance=21.15, speed=speed),
        TurnAction(degrees=90.0, turn_radius=1.0),
        LinearMoveAction(distance=5.15, speed=speed),
        TurnAction(degrees=-88.0, turn_radius=1.0),
        LinearMoveAction(distance=1.6, speed=speed),
        LiftAction(height=0.5, speed=0.2),
        LinearMoveAction(distance=3.0, speed=-speed),
        TurnAction(degrees=-180.0, turn_radius=2.0),
        TurnAction(degrees=90.0, turn_radius=2.0),
        LinearMoveAction(distance=1.62, speed=speed),
        LiftAction(height=0.3, speed=0.2),
        LinearMoveAction(distance=3.1, speed=-speed),
        
    ],
    "forklift_3": [
        WaitAction(duration=157.0),
        TurnAction(degrees=90.0, turn_radius=2.0),
        LinearMoveAction(distance=20.5, speed=speed),
        TurnAction(degrees=-180.0, turn_radius=2.0),
        LinearMoveAction(distance=0.6, speed=speed),
        TurnAction(degrees=90.0, turn_radius=1.0),
        LiftAction(height=1.85, speed=0.2),
        LinearMoveAction(distance=3.0, speed=speed),
        LiftAction(height=2.0, speed=0.2),
        LinearMoveAction(distance=3.8, speed=-speed),
        LiftAction(height=0.3, speed=0.2),
        TurnAction(degrees=-90.0, turn_radius=1.0),
        LinearMoveAction(distance=28.0, speed=speed+(-20)),
    ]
}

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def get_yaw_from_rotation(rotation):
    """
    Webots rotation = [ax, ay, az, angle]
    For ground robots, yaw ≈ az * angle
    """
    _, _, az, angle = rotation
    return normalize_angle(az * angle)

def set_pallet_transported(robot: Supervisor, pallet_field_name: str):
    pallet = robot.getFromDef(pallet_field_name)
    if pallet:
        transported_field = pallet.getField("transported")
        transported_field.setSFBool(True)
        print(f"Set {pallet_field_name} as transported.")
    else:
        print(f"Error: {pallet_field_name} not found to set as transported.")


def ackermann_rear_wheel_angles(wheelbase, track_width, turn_radius):
    """
    Rear-wheel Ackermann angles (radians)
    """
    inner = math.atan(wheelbase / (turn_radius - track_width / 2))
    outer = math.atan(wheelbase / (turn_radius + track_width / 2))
    return inner, outer

def turn_n_degrees_rear_steer(
    robot: Supervisor,
    rotation_field: Field,
    target_deg: float,
    turn_radius: float=1.0,
):
    """
    Turn the robot by N degrees using rear-wheel steering.
    """

    # Initial yaw
    start_yaw = get_yaw_from_rotation(rotation_field.getSFRotation())
    target_yaw = normalize_angle(start_yaw + math.radians(target_deg))

    # Compute steering angles
    delta_inner, delta_outer = ackermann_rear_wheel_angles(
        wheel_base, track_width, turn_radius
    )

    # Left turn
    if target_deg > 0:
        # print(f"Setting rear wheel angles for left turn: inner={-delta_inner}, outer={-delta_outer}")
        rear_left_steer_motor.setPosition(-delta_inner)
        rear_right_steer_motor.setPosition(-delta_outer)
    else:
        # print(f"Setting rear wheel angles for right turn: inner={delta_inner}, outer={delta_outer}")
        rear_left_steer_motor.setPosition(delta_outer)
        rear_right_steer_motor.setPosition(delta_inner)

    # Drive forward
    front_left_motor.setVelocity(speed)
    front_right_motor.setVelocity(speed)

    # Control loop
    while robot.step(timestep) != -1:
        current_yaw = get_yaw_from_rotation(rotation_field.getSFRotation())
        yaw_error = normalize_angle(target_yaw - current_yaw)

        # print(f"Current yaw: {math.degrees(current_yaw):.2f} deg, Target yaw: {math.degrees(target_yaw):.2f} deg, Error: {math.degrees(yaw_error):.2f} deg")

        if abs(yaw_error) < math.radians(1.0):  # 1 degree tolerance
            break

    # Stop + straighten
    front_left_motor.setVelocity(0.0)
    front_right_motor.setVelocity(0.0)
    rear_left_steer_motor.setPosition(0.0)
    rear_right_steer_motor.setPosition(0.0)

def lift(slider_motor: Motor, pos_sensor: PositionSensor, target_height: float, speed: float=0.2):
    if target_height < min_forklift_height or target_height > max_forklift_height:
        print(f"Error: Target height {target_height} is out of bounds.")
        return

    slider_motor.setVelocity(speed)
    slider_motor.setPosition(target_height)

    while robot.step(timestep) != -1:
        current_height = pos_sensor.getValue()
        height_error = target_height - current_height
        
        print(f"Current height: {current_height:.3f}, Target height: {target_height:.3f}, Error: {height_error:.3f}")

        if abs(height_error) < 0.01:
            break


def linear_move(front_left_motor: Motor,
                front_right_motor: Motor,
                translation_field: Field,
                target_distance: float,
                speed: float = -2.0,
                tol: float = 0.01):

    start_pos = translation_field.getSFVec3f()

    rear_left_steer_motor.setPosition(0.0)
    rear_right_steer_motor.setPosition(0.0)

    front_left_motor.setVelocity(speed)
    front_right_motor.setVelocity(speed)

    while robot.step(timestep) != -1:
        current_pos = translation_field.getSFVec3f()

        dx = current_pos[0] - start_pos[0]
        dy = current_pos[1] - start_pos[1]

        traveled = math.hypot(dx, dy)
        distance_error = target_distance - traveled

        # print(f"Traveled: {traveled:.3f}, Error: {distance_error:.3f}")

        if distance_error <= tol:
            break

    front_left_motor.setVelocity(0.0)
    front_right_motor.setVelocity(0.0)

def straighten_wheels(rear_left_steer_motor: Motor, rear_right_steer_motor: Motor, left_steer_ps: PositionSensor, right_steer_ps: PositionSensor, tol: float = 0.01):
    rear_left_steer_motor.setPosition(0.0)
    rear_right_steer_motor.setPosition(0.0)

    while robot.step(timestep) != -1:
        left_steer_angle = left_steer_ps.getValue()
        right_steer_angle = right_steer_ps.getValue()

        # print(f"Left steer angle: {math.degrees(left_steer_angle):.2f} deg, Right steer angle: {math.degrees(right_steer_angle):.2f} deg")

        if abs(left_steer_angle) < tol and abs(right_steer_angle) < tol:
            break

def stop(front_left_motor: Motor, front_right_motor: Motor, rear_left_steer_motor: Motor, rear_right_steer_motor: Motor):
    print("Stopping robot and straightening wheels.")
    front_left_motor.setVelocity(0.0)
    front_right_motor.setVelocity(0.0)
    rear_left_steer_motor.setPosition(0.0)
    rear_right_steer_motor.setPosition(0.0)

def wait(duration: float):
    start_time = robot.getTime()
    while robot.getTime() - start_time < duration:
        robot.step(timestep)

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

admin = robot.getFromDef("ADMIN")

#make sure that the robot is a supervisor
print(f"Checking if robot: {robot.getName()} has supervisor capabilities...")
if not robot.supervisor:
    print("Error: This controller requires supervisor capabilities.")
    exit(1)

# Motors
front_left_motor: Motor = robot.getDevice("front_left_motor")
front_right_motor: Motor = robot.getDevice("front_right_motor")
slider_motor: Motor = robot.getDevice("slider_motor")
rear_left_steer_motor: Motor = robot.getDevice("rear_left_steer_motor")
rear_right_steer_motor: Motor = robot.getDevice("rear_right_steer_motor")


front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
# slider_motor.setPosition(float('inf'))
rear_left_steer_motor.setPosition(float('inf'))
rear_right_steer_motor.setPosition(float('inf'))

front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
slider_motor.setVelocity(0.0)
slider_motor.setPosition(min_forklift_height)
rear_left_steer_motor.setVelocity(speed)
rear_right_steer_motor.setVelocity(speed)
rear_left_steer_motor.setPosition(0.0)
rear_right_steer_motor.setPosition(0.0)

slider_position_sensor: PositionSensor = robot.getDevice("slider_position_sensor")
left_steer_ps: PositionSensor = robot.getDevice("left_steer_ps")
right_steer_ps: PositionSensor = robot.getDevice("right_steer_ps")

slider_position_sensor.enable(timestep)
left_steer_ps.enable(timestep)
right_steer_ps.enable(timestep)

translation_field = robot.getFromDef(robot.getName()).getField("translation")
rotation_field = robot.getFromDef(robot.getName()).getField("rotation")

actions = PLANS.get(robot.getName(), [])
print(f"Loaded {len(actions)} actions for {robot.getName()}.")
print(actions)

# exit(0)  # TEMP: Remove this line to run the full plan

while robot.step(timestep) != -1:
    if len(actions) == 0:
        print("All actions completed.")
        break

    action = actions[0]
    print(f"Executing action: {action}")

    if isinstance(action, LinearMoveAction):
        # print(f"Executing LinearMoveAction: distance={action.distance}, speed={action.speed}, tol={action.tol}")
        straighten_wheels(rear_left_steer_motor, rear_right_steer_motor, left_steer_ps, right_steer_ps, tol=0.01)
        linear_move(front_left_motor, front_right_motor, translation_field, action.distance, action.speed, action.tol)
        # stop(front_left_motor, front_right_motor, rear_left_steer_motor, rear_right_steer_motor)
    elif isinstance(action, TurnAction):
        # print(f"Executing TurnAction: degrees={action.degrees}, turn_radius={action.turn_radius}")
        turn_n_degrees_rear_steer(robot, rotation_field, action.degrees, action.turn_radius)
        # stop(front_left_motor, front_right_motor, rear_left_steer_motor, rear_right_steer_motor)
    elif isinstance(action, LiftAction):
        # print(f"Executing LiftAction: height={action.height}, speed={action.speed}")
        lift(slider_motor, slider_position_sensor, action.height, action.speed)
        # stop(front_left_motor, front_right_motor, rear_left_steer_motor, rear_right_steer_motor)
    elif isinstance(action, SetPalletTransportedAction):
        # print(f"Executing SetPalletTransportedAction")
        current_pallet_field = admin.getField("current_pallet").getSFString()
        set_pallet_transported(robot, current_pallet_field)
    elif isinstance(action, WaitAction):
        # print(f"Executing WaitAction: duration={action.duration}")
        wait(action.duration)

    actions.pop(0)
    print(f"Actions Left: {actions}")