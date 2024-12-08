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
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)

        self.get_logger().info("Stepper motor node initialized")

    def rotate_motor(self, steps, direction, delay):
        GPIO.output(self.DIR_PIN, direction)
        for _ in range(steps):
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(delay)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorNode()

    try:
        node.get_logger().info("Rotating forward...")
        node.rotate_motor(steps=100, direction=GPIO.HIGH, delay=0.001)
        time.sleep(5)
        node.get_logger().info("Rotating backward...")
        node.rotate_motor(steps=100, direction=GPIO.LOW, delay=0.001)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()