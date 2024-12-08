import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')

        # GPIO setup
        self.DIR_PIN = 20
        self.STEP_PIN = 21
        self.DIR2_PIN = 22
        self.STEP2_PIN = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR2_PIN, GPIO.OUT)
        GPIO.setup(self.STEP2_PIN, GPIO.OUT)

        self.get_logger().info("Stepper motor node initialized")

    def rotate_motor(self, steps, direction, delay):
        GPIO.output(self.DIR_PIN, direction)
        for _ in range(steps):
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(delay)

    def rotate_motor2(self, steps, direction, delay):
        GPIO.output(self.DIR2_PIN, direction)
        for _ in range(steps):
            GPIO.output(self.STEP_PIN2, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.STEP_PIN2, GPIO.LOW)
            time.sleep(delay)


    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()