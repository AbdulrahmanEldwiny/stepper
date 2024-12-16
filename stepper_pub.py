import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
import RPi.GPIO as GPIO
import time

class StepperControlNode(Node):
    def __init__(self):
        super().__init__('Stepper_control_node')
        self.DIR_PIN = 20
        self.STEP_PIN = 21
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)

        self.publisher_ = self.create_publisher(Int64, '/stepper_controller/command', 10)
        self.timer = self.create_timer(0.18, self.timer_callback)

        self.angle = 0.0
        self.increment = 1
        self.max_steps = 100
        self.steps_completed = 0
        self.direction = 1
        self.STEPS_180_DEGREES = 100

        self.get_logger().info("Stepper Control Node Initialized")

    def timer_callback(self):
        if self.direction == 1:
            self.direction = 0
        else:
            self.direction = 1
        self.rotate_stepper()

    def rotate_stepper(self):
        ros_steps = Int64()
        ros_steps.data = self.steps_completed
        steps_to_rotate = self.STEPS_180_DEGREES
        GPIO.output(self.DIR_PIN, GPIO.HIGH if self.direction == 1 else GPIO.LOW)
        for steps_completed in range(steps_to_rotate):
            self.publisher_.publish(ros_steps)
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(0.001)
            
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(0.001)
            steps_completed += 1
            ros_steps.data = self.steps_completed

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StepperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
