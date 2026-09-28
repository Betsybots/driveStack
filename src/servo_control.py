import time
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory

# Initialize the lgpio pin factory
factory = LGPIOFactory()

# Assign the servo to GPIO 18 (Physical Pin 12)
servo = Servo(18, pin_factory=factory, min_pulse_width=1/1000, max_pulse_width=2/1000)

try:
    print("Moving servo... Press Ctrl+C to stop.")
    while True:
        print("Center (0°)")
        servo.value = 0
        time.sleep(1)

        print("Max (+90°)")
        servo.value = 1
        time.sleep(1)

        print("Center (0°)")
        servo.value = 0
        time.sleep(1)

        print("Min (-90°)")
        servo.value = -1
        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping servo...")
    servo.detach()
