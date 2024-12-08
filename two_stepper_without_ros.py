import RPi.GPIO as GPIO
import time


DIR_PIN = 20  
STEP_PIN = 21
DIR2_PIN = 22
STEP2_PIN = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR2_PIN2, GPIO.OUT)
GPIO.setup(STEP2_PIN, GPIO.OUT)

def rotate_motor(steps, direction, delay):
    GPIO.output(DIR_PIN, direction)
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)
        
def rotate_motor2(self, steps, direction, delay):
        GPIO.output(self.DIR2_PIN, direction)
        for _ in range(steps):
            GPIO.output(self.STEP_PIN2, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.STEP_PIN2, GPIO.LOW)
            time.sleep(delay)


try:
        # first rotating
        node.get_logger().info("Rotating forward first motor...")
        node.rotate_motor(steps=100, direction=GPIO.HIGH, delay=0.001)
        time.sleep(1)
        node.get_logger().info("Rotating backward first motor...")
        node.rotate_motor(steps=100, direction=GPIO.LOW, delay=0.001)
        time.sleep(10)

        # second rotating
        node.get_logger().info("Rotating forward  second motor...")
        node.rotate_motor2(steps=50, direction=GPIO.HIGH, delay=0.001)
        node.get_logger().info("Rotating forward first motor...")
        node.rotate_motor(steps=17, direction=GPIO.HIGH, delay=0.001)
        time.sleep(1)
        node.get_logger().info("Rotating backward second motor...")
        node.rotate_motor2(steps=50, direction=GPIO.LOW, delay=0.001)
        node.get_logger().info("Rotating backward first motor...")
        node.rotate_motor(steps=17, direction=GPIO.LOW, delay=0.001)

finally:
    GPIO.cleanup()
