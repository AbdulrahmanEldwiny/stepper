import rclpy                       # ros2 python liberary
from rclpy.node import Node        # ros2 Node liberary
from std_msgs.msg import Float64   # The data type used to publish the angle in a ROS topic is std_msgs/Float64
import RPi.GPIO as GPIO            # library used to control GPIO pins on Raspberry Pi.
import time                        # liberary deals with delays

class StepperControlNode(Node):
    def __init__(self):
        super().__init__('Stepper_control_node')

        self.DIR_PIN = 20    # GPIO20
        self.STEP_PIN = 21   # GPIO21
        GPIO.setmode(GPIO.BCM)   # Broadcom method
        GPIO.setup(self.DIR_PIN, GPIO.OUT)  # make the direction pin as  output
        GPIO.setup(self.STEP_PIN, GPIO.OUT) # make the pulse pin as output
        
        self.publisher_ = self.create_publisher(Float64, '/stepper_controller/command', 10)  #Creates a publisher that sends values of type Float64 to the topic /r2d2_head_controller/command.
        self.timer = self.create_timer(0.03, self.timer_callback)   #Creates a timer that executes the function timer_callback every 30 milliseconds

        self.angle = 0.0    # A variable to store the current angle of the motor
        self.increment = 1  # The value by which the step(angle) increases each time the motor is moved
        self.max_steps = 100   # The maximum number of steps the motor can take before switching direction
        self.steps_completed = 0  # A variable to store the number of steps the motor has completed
        self.direction = 1    # A variable to determine the direction of motion. If its value is 1, the step(angle) increases; if it's -1, the step(angle) decreases.
        self.STEPS_180_DEGREES = 100  #The number of steps required to move the motor 180 degrees. It is set here as 100 steps.

        self.get_logger().info("Lidar Control Node Initialized")

    def timer_callback(self):
        self.publisher_.publish(Float64(data=self.steps_completed))   # Publishes the number of completed steps to the topic /r2d2_head_controller/command
        
        if self.direction == 1:     # If the value is 1, it means the direction is to increase the number of steps
            self.steps_completed += self.increment    # Increases the number of steps by the increment value
            if self.steps_completed >= self.max_steps:  # If the maximum number of steps (max_steps) is reached, the direction is changed to -1 (decreasing)
                self.direction = -1    # If the value is -1, it means the direction is to decrease the number of steps
        else:    #  If the value is -1, it means the direction is to decrease the number of steps
            self.steps_completed -= self.increment    # Decreases the number of steps
            if self.steps_completed <= 0:      # If the steps reach zero, the direction is changed to 1 (increasing).
                self.direction = 1
        
        self.rotate_stepper()    # Calls the rotate_stepper function to rotate the motor

    def rotate_stepper(self):    # function rotate the stepper
        steps_to_rotate = self.STEPS_180_DEGREES  # Sets the number of steps the motor should take, which is 100 in this case
        
        GPIO.output(self.DIR_PIN, GPIO.HIGH if self.direction == 1 else GPIO.LOW)    # Sets the motor's direction based on the value in self.direction.
# If it's 1, a HIGH signal is sent (rotation in one direction), and if it's -1, a LOW signal is sent (rotation in the opposite direction).
        for _ in range(steps_to_rotate):    #A loop that repeats the operation based on the number of steps the motor needs to take.

            GPIO.output(self.STEP_PIN, GPIO.HIGH)    # Sends a pulse to activate the step
            time.sleep(0.001)     # A 1-millisecond delay between each step
            GPIO.output(self.STEP_PIN, GPIO.LOW)    # Sends a pulse to stop the step
            time.sleep(0.001)    # Another delay between each step

    def destroy_node(self):
        GPIO.cleanup()    # Cleans up the pins at the end of the program after finishing
        super().destroy_node()    # Calls the destroy_node function from the base class Node to clean up the node

def main(args=None):
    rclpy.init(args=args)    # Initializes ROS2
    node = StepperControlNode()    #  Creates an object that contains the StepperControlNode

    try:
        rclpy.spin(node)    # Keeps the node running
    except KeyboardInterrupt:
        pass
    finally:    # Finally, the node is cleaned up after finishing
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()