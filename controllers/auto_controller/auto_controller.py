from controller import Robot, Motor, Supervisor, Field, PositionSensor
import math

# Constants
speed = -2.0
max_steer_angle = 1.0 
min_forklift_height = 0.001
max_forklift_height = 2.0
wheel_base = 0.83
track_width = 0.63

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


def ackermann_rear_wheel_angles(wheelbase, track_width, turn_radius):
    """
    Rear-wheel Ackermann angles (radians)
    """
    inner = math.atan(wheelbase / (turn_radius - track_width / 2))
    outer = math.atan(wheelbase / (turn_radius + track_width / 2))
    return inner, outer

def turn_n_degrees_rear_steer(
    robot: Supervisor,
    translation_field: Field,
    rotation_field: Field,
    target_deg: float,
    # wheelbase=0.35,
    # track_width=0.25,
    turn_radius: float=1.0,
    # drive_speed=2.0
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
        rear_left_steer_motor.setPosition(-delta_inner)
        rear_right_steer_motor.setPosition(-delta_outer)
    else:
        rear_left_steer_motor.setPosition(delta_outer)
        rear_right_steer_motor.setPosition(delta_inner)

    # Drive forward
    front_left_motor.setVelocity(speed)
    front_right_motor.setVelocity(speed)

    # Control loop
    while robot.step(timestep) != -1:
        current_yaw = get_yaw_from_rotation(rotation_field.getSFRotation())
        yaw_error = normalize_angle(target_yaw - current_yaw)

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


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

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
slider_motor.setPosition(float('inf'))
rear_left_steer_motor.setPosition(float('inf'))
rear_right_steer_motor.setPosition(float('inf'))

front_left_motor.setVelocity(speed)
front_right_motor.setVelocity(speed)
slider_motor.setVelocity(0.0)
slider_motor.setPosition(min_forklift_height)
rear_left_steer_motor.setVelocity(speed)
rear_right_steer_motor.setVelocity(speed)
rear_left_steer_motor.setPosition(0.0)
rear_right_steer_motor.setPosition(0.0)

slider_position_sensor: PositionSensor = robot.getDevice("slider_position_sensor")
slider_position_sensor.enable(timestep)

translation_field = robot.getFromDef(robot.getName()).getField("translation")
rotation_field = robot.getFromDef(robot.getName()).getField("rotation")

turned = False
while robot.step(timestep) != -1:
    # current_pos = translation_field.getSFVec3f()
    # current_rotation = rotation_field.getSFRotation()

    # print(f"Current position: {current_pos}")
    # print(f"Current rotation: {current_rotation}")

    if not turned:
        print("Turning 90 degrees...")
        turn_n_degrees_rear_steer(
            robot,
            translation_field,
            rotation_field,
            target_deg=-90,
            # wheelbase=wheel_base,
            # track_width=track_width,
            turn_radius=1.0,
            # drive_speed=-2.0
        )
        turned = True
    if turned:
        print("Lifting forklift to 1.5 meters...")
        lift(slider_motor, slider_position_sensor, target_height=1.5, speed=0.2)
        turned = False 


