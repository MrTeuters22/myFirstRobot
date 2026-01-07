from wpimath.controller import SimpleMotorFeedforward

# Gains (kS, kV, kA) are usually determined via system identification (SysID)
kS = 0.5  # Static friction gain
kV = 2.0  # Velocity gain
kA = 0.2  # Acceleration gain

feedforward = SimpleMotorFeedforward(kS, kV, kA)

# Calculate the required voltage for a desired velocity and acceleration
# Units depend on the units used for the gains
desired_velocity = 10.0  # units/second
desired_acceleration = 0.0 # units/second^2 (if accelerating, provide a value)

# The output is the control effort (e.g., voltage)
volts = feedforward.calculate(desired_velocity, desired_acceleration)

# In a full system, you would send this voltage to the motor controller
# motor.setVoltage(volts)
print(f"Calculated feedforward voltage: {volts} V")
