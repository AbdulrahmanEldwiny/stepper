import RPi.GPIO as GPIO
import time


DIR_PIN = 20  
STEP_PIN = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)

def rotate_motor(steps, direction, delay):
    GPIO.output(DIR_PIN, direction)
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)

try:
    print("Rotating clockwise...")
    rotate_motor(steps=200, direction=GPIO.HIGH, delay=0.001)
    time.sleep(5)
    print("Rotating counterclockwise...")
    rotate_motor(steps=200, direction=GPIO.LOW, delay=0.001)

finally:
    GPIO.cleanup()