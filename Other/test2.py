#!/usr/bin/env pybricks-micropython
import from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
lcolorSensor = ColorSensor(Port.S1)
rcolorSensor = ColorSensor(Port.S2)
gyroSensor = GyroSensor(Port.S3,positive_direction=Direction.CLOCKWISE)
lMotor = Motor(Port.B,positive_direction=Direction.CLOCKWISE,gears=None)
rMotor = Motor(Port.C,positive_direction=Direction.CLOCKWISE,gears=None)
robot = DriveBase(lMotor, rMotor, wheel_diameter=55.5, axle_track=104)

BLACK=7
WHITE=37
threshold = (BLACK+WHITE)/2 
DRIVE_SPEED = 100


# Go forward and backwards for one meter.
robot.straight(1000)
ev3.speaker.beep()

robot.straight(-1000) and ev3.speaker.beep()

# Turn clockwise by 360 degrees and back again.
robot.turn(360)
ev3.speaker.beep()

robot.turn(-360)
ev3.speaker.beep()

# Stop the program.
ev3.speaker.say("Program complete")