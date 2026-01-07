# Conceptual Python code for a PID drive straight function
# Assumes gyro and motor objects are already defined and reset

# PID Gains
Kp = 20
Ki = 0.5
Kd = 10

# Target angle
target_angle = 0

# Variables for integral and previous error
last_error = 0
integral = 0

# Loop to control the robot
while True:
    # Read sensor
    current_angle = gyro.angle()

    # Calculate error
    error = target_angle - current_angle
    integral = integral + error
    derivative = error - last_error

    # Calculate PID output
    turn = (Kp * error) + (Ki * integral) + (Kd * derivative)
    last_error = error

    # Apply control to motors
    left_speed = 150 + turn
    right_speed = 150 - turn

    left_motor.dc(left_speed)
    right_motor.dc(right_speed)

    # Wait a short time
    wait(10)

    # Break the loop if a condition is met (e.g., reached target distance)
    # if robot.distance() > 1000:
    #     break