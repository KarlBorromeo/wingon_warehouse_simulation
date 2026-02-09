from controller import Robot, Keyboard

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Keyboard
keyboard = Keyboard()
keyboard.enable(timestep)

# Speed (same idea as C code)
speed = 0.05

# Get motors (names MUST match your robot)
left_motor = robot.getDevice("left motor")
right_motor = robot.getDevice("right motor")

# Velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

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
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)

    elif key == ord('A'):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)

    elif key == ord('S'):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(-speed)

    elif key == ord('D'):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)
