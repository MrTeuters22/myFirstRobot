#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.ev3devices import ColorSensor as PybricksColorSensor
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Direction
from pybricks.tools import wait

from threading import Thread
from pybricks.parameters import SoundFile, Color, Port
# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C) 

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
# Initialize reflectance sensors
lcolorSensor = PybricksColorSensor(Port.S1)
rcolorSensor = PybricksColorSensor(Port.S4)
# Calculate the light threshold. Choose values based on your measurements.
BLACK = 10
WHITE = 70

# Set threshold
threshold = (BLACK + WHITE) / 2
# Proportional gain for line following
Kp = 6
main = True
while True:
    # Read the color sensors
    left_reflection = lcolorSensor.reflection()
    right_reflection = rcolorSensor.reflection()
    
    # Calculate the error
    error = left_reflection - right_reflection
    
    # Calculate the turn rate
    turn_rate = Kp * error
    
    # Drive the robot
    robot.drive(100, turn_rate)
    
    # Small delay to prevent overwhelming the CPU
    wait(10)







