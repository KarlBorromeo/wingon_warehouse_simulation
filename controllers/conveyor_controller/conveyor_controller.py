from controller import Robot, Motor

# create the Robot instance.
robot = Robot()
timestep = int(robot.getBasicTimeStep())

belt_motor: Motor = robot.getDevice("belt_motor")
belt_motor.setPosition(float('inf'))  # Set to velocity control mode
belt_motor.setVelocity(0.0)  # Start with the belt stopped

while robot.step(timestep) != -1:
    belt_motor.setVelocity(0.3)  # Set belt speed to 2.0 (adjust as needed)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
