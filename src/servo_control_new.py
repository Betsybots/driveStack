import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# Initialize the pigpio factory
factory = PiGPIOFactory()

# Assign the servo to GPIO 18 (Physical Pin 12)
servo = Servo(18, pin_factory=factory, min_pulse_width=1/1000, max_pulse_width=2/1000)

try:
    print("Servo control started using pigpiod. Press Ctrl+C to exit.")
    while True:

        print("Center (0°)")
        servo.value = 0
        time.sleep(1)
        for i in range(0,6):
            servo.value = i * 0.1
            print(servo.value)
            time.sleep(1)
        for i in range(6,0, -1):
            servo.value = i* 0.1
            print(servo.value)
            time.sleep(1)
        for i in range(0,6):
            servo.value = i * -0.1
            print(servo.value)
            time.sleep(1)
        for i in range(6,0, -1):
            servo.value = i* -0.1
            print(servo.value)
            time.sleep(1)

        #print("Min (-90°)")
        #servo.value = 0
        #time.sleep(1)

except KeyboardInterrupt:
    print("\nStopping...")
    servo.detach()

