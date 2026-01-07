#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

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

# Set the language for speech
# Example: English
#ev3.speaker.set_speech_options(language='en')

# Make the robot speak
#ev3.speaker.say('Hello, world!')


# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Set up the gyro sensor.
# gyroSensor = GyroSensor(Port.S2, positive_direction=Direction.CLOCKWISE) 

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Initialize reflectance sensors
lcolorSensor = PybricksColorSensor(Port.S1)
rcolorSensor = PybricksColorSensor(Port.S4)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 10
WHITE = 70
# Threshold has been replaced with feed forward
threshold = (BLACK + WHITE) / 2

# PID Gains 1.2, 0.012, 200
Kp = 6
Ki = 0.00005
Kd = 18
Kf = 0

# Variables for Error
error = 0

# Variables for integral and previous error
last_error = 0
integral = 0
integral_range = 5
integral_rate = 0.1

# feedforward term
ff = 0

# Variables for Derivative
derivative = 0

# Set the drive speed at n millimeters per second.
DRIVE_SPEED = 90

# Set the target reflectance value.
#target_reflectance = 10 # Adjust this value based on your line color

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
# PROPORTIONAL_GAIN = 1.2

# Defining feedforward term based on sensor values
def calculate_feedforward(lref, rref):
    # Example: simple average of sensor readings
    return (lref + rref) / 2   / 100  # Normalize to a smaller range    

def twenty_tones_thread():
    for j in range(0,20):
        ev3.speaker.play_tone(440 + i * 20, 100)
        wait(50)
        
def PID_calculation_thread():
    global error, integral, last_error, derivative, ff
    error = (lcolorSensor.reflection() - rcolorSensor.reflection())
    ff = calculate_feedforward(lcolorSensor.reflection(), rcolorSensor.reflection())  
    integral = integral + error
    derivative = error - last_error
    last_error = error
    
def turn_rate_thread():
     turn_rate = ((error*Kp) + (integral*Ki) + (derivative*Kd) + (ff*Kf))
        

while True:
    lreflectance = lcolorSensor.reflection()
    rreflectance = rcolorSensor.reflection()
    # Start threads for PID calculation and turn rate calculation
    pid_thread = Thread(target=PID_calculation_thread)
    turn_thread = Thread(target=turn_rate_thread)
    pid_thread.start()
    turn_thread.start()
    pid_thread.join()
    turn_thread.join()
    # Drive the robot with the calculated turn rate
    robot.drive(DRIVE_SPEED, turn_rate)
    # You can wait for a short time or do other things in this loop.
    wait(10)