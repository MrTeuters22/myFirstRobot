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
ev3.speaker.set_speech_options(language='en')
# Make the robot speak
ev3.speaker.say('Hello, world!')    
# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C) 
# Set up the gyro sensor.
gyroSensor = GyroSensor(Port.S2, positive_direction=Direction.CLOCKWISE) 
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
target_gyro = 0     
# PID Gains 1.2, 0.012, 200
Kp = 6
Ki = 0.00005
Kd = 18
Kf = 0
# PID for Gyro
GYRO_Kp = 4
GYRO_Ki = 0.0001
GYRO_Kd = 12            
# Variables for Error
error = 0
gyro_error = 0      
# Variables for integral and previous error
last_error = 0
last_gyro_error = 0
integral = 0
integral_rate = 0.1
integral_range = 50 
gyro_integral = 0
gyro_integral_rate = 0.1
gyro_integral_range = 50        
# Robot speed
BASE_SPEED = 100    
# Main loop
while True:
    # Read the color sensors
    left_reflection = lcolorSensor.reflection()
    right_reflection = rcolorSensor.reflection()
    
    # Calculate the error from the threshold
    error = left_reflection - right_reflection
    
    # Calculate the integral
    if abs(error) < integral_range:
        integral += error * integral_rate
    else:
        integral = 0
    
    # Calculate the derivative
    derivative = error - last_error
    
    # PID control for line following
    turn_rate = (Kp * error) + (Ki * integral) + (Kd * derivative)
    
    # Gyro control
    gyro_error = target_gyro - gyroSensor.angle()
    
    # Calculate the gyro integral
    if abs(gyro_error) < gyro_integral_range:
        gyro_integral += gyro_error * gyro_integral_rate
    else:
        gyro_integral = 0
    
    # Calculate the gyro derivative
    gyro_derivative = gyro_error - last_gyro_error
    
    # PID control for gyro
    gyro_correction = (GYRO_Kp * gyro_error) + (GYRO_Ki * gyro_integral) + (GYRO_Kd * gyro_derivative)
    
    # Combine line following and gyro correction
    total_turn_rate = turn_rate + gyro_correction
    
    # Drive the robot
    robot.drive(BASE_SPEED, total_turn_rate)
    
    # Update last errors
    last_error = error
    last_gyro_error = gyro_error
    
    # Small delay to prevent overload
    wait(10)    
    
# Stop the robot (this line will never be reached in the current loop)
robot.stop()


