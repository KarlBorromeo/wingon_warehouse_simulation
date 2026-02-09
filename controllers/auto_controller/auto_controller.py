from controller import Robot, Motor, Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

#make sure that the robot is a supervisor
print(f"Checking if robot: {robot.getName()} has supervisor capabilities...")
if not robot.supervisor:
    print("Error: This controller requires supervisor capabilities.")
    exit(1)


# Constants
speed = -2.0
max_steer_angle = 1.0 
min_forklift_height = 0.001
max_forklift_height = 0.1

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

translation_field = robot.getFromDef("FORKLIFT").getField("translation")
rotation_field = robot.getFromDef("FORKLIFT").getField("rotation")

while robot.step(timestep) != -1:
    current_pos = translation_field.getSFVec3f()
    current_rotation = rotation_field.getSFRotation()

    print(f"Current position: {current_pos}")
    print(f"Current rotation: {current_rotation}")


