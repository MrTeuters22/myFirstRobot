from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sound import Sound

# /Users/ateut/Desktop/myFirstRobot/test.py

def main():
    motor_b = LargeMotor(OUTPUT_B)
    motor_c = LargeMotor(OUTPUT_C)
    sound = Sound()

    # run motor B for 1 second
    motor_b.on_for_seconds(SpeedPercent(50), 1)
    # run motor C for 1 second
    motor_c.on_for_seconds(SpeedPercent(50), 1)

    # beep twice
    sound.beep()
    sound.beep()

if __name__ == "__main__":
    main()