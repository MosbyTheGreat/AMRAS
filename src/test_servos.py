import time
from adafruit_servokit import ServoKit
import adafruit_motor.servo
import RPi.GPIO as GPIO

# Relays
GPIO.setmode(GPIO.BCM)

Pin1 = 24
Pin2 = 23
Pin3 = 22
Pin4 = 27

GPIO.setup(Pin1, GPIO.OUT)
GPIO.setup(Pin2, GPIO.OUT)
GPIO.setup(Pin3, GPIO.OUT)
GPIO.setup(Pin4, GPIO.OUT)

GPIO.output(Pin1, GPIO.LOW)
GPIO.output(Pin2, GPIO.LOW)
GPIO.output(Pin3, GPIO.LOW)
GPIO.output(Pin4, GPIO.LOW)

GPIO.output(Pin1, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(Pin1, GPIO.LOW)
time.sleep(2.0)

GPIO.output(Pin2, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(Pin2, GPIO.LOW)
time.sleep(2.0)

GPIO.output(Pin3, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(Pin3, GPIO.LOW)
time.sleep(2.0)

GPIO.output(Pin4, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(Pin4, GPIO.LOW)
time.sleep(2.0)

# Servos
kit = ServoKit(channels=16)

kit.servo[0].angle = 0
kit.servo[1].angle = 0
time.sleep(2.0)

kit.servo[0].angle = 90
kit.servo[1].angle = 90
time.sleep(2.0)

kit.servo[0].angle = 180
kit.servo[1].angle = 180
time.sleep(2.0)

kit.servo[0].angle = 90
kit.servo[1].angle = 90
