from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.ev3devices import ColorSensor as PybricksColorSensor
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Direction

# Initialize the EV3 Brick.
ev3 = EV3Brick()
# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
# Initialize the sensors.
colorSensor = PybricksColorSensor(Port.S1)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
# Set the Drive Speed.
DRIVE_SPEED = 100

# Go forward and backwards for one meter.
robot.straight(1000)
ev3.speaker.beep()
robot.straight(-1000) and ev3.speaker.beep()

robot.stop()