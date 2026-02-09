from controller import Robot, Keyboard

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Keyboard
keyboard = Keyboard()
keyboard.enable(timestep)

# Speed (same idea as C code)
speed = 2.0

# Get motors (names MUST match your robot)
front_left_motor = robot.getDevice("front_left_motor")
front_right_motor = robot.getDevice("front_right_motor")
rear_left_motor = robot.getDevice("rear_left_motor")
rear_right_motor = robot.getDevice("rear_right_motor")
slider_motor = robot.getDevice("slider_motor")
front_left_steer_motor = robot.getDevice("front_left_steer_motor")
front_right_steer_motor = robot.getDevice("front_right_steer_motor")


# Velocity control mode
front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))
slider_motor.setPosition(float('inf'))
front_left_steer_motor.setPosition(float('inf'))
front_right_steer_motor.setPosition(float('inf'))

front_left_motor.setVelocity(speed)
front_right_motor.setVelocity(speed)
rear_left_motor.setVelocity(speed)
rear_right_motor.setVelocity(speed)
slider_motor.setVelocity(0.0)
front_left_steer_motor.setVelocity(speed)
front_right_steer_motor.setVelocity(speed)
front_left_steer_motor.setPosition(0.0)
front_right_steer_motor.setPosition(0.0)

print("Select the 3D window and use the keyboard:")
print(" W: forward")
print(" A: turn left")
print(" S: backward")
print(" D: turn right")
print("---------------------------------------------")

# Main loop
while robot.step(timestep) != -1:
    key = keyboard.getKey()

    if key == ord('W'):
        front_left_motor.setVelocity(speed)
        front_right_motor.setVelocity(speed)
        rear_left_motor.setVelocity(speed)
        rear_right_motor.setVelocity(speed)


    elif key == ord('A'):
        front_left_steer_motor.setPosition(1.0)
        front_right_steer_motor.setPosition(1.0)

    elif key == ord('S'):
        front_left_motor.setVelocity(-speed)
        rear_left_motor.setVelocity(-speed)
        front_right_motor.setVelocity(-speed)
        rear_right_motor.setVelocity(-speed)

    elif key == ord('D'):
        front_left_steer_motor.setPosition(-1.0)
        front_right_steer_motor.setPosition(-1.0)

    elif key == ord('Q'):
        slider_motor.setVelocity(0.2)
        slider_motor.setPosition(2.0)

    elif key == ord('E'):
        slider_motor.setVelocity(0.2)
        slider_motor.setPosition(0.0)

    elif key == ord('X'):
        front_left_motor.setVelocity(0.0)
        front_right_motor.setVelocity(0.0)
        rear_left_motor.setVelocity(0.0)
        rear_right_motor.setVelocity(0.0)
        front_left_steer_motor.setPosition(0.0)
        front_right_steer_motor.setPosition(0.0)
