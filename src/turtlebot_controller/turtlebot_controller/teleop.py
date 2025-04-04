#importing all the necessary libraries and modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

# defining the step sizes for linear and angular velocities
LIN_VEL_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 0.1

# defining the Teleop class
class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for the Twist message
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Initializing linear and angular velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    # defining the get_key function to capture key presses from the terminal
    def get_key(self):
        """ Capture key presses from the terminal """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """ Main loop for key detection and publishing velocity commands """
        self.get_logger().info("""
        Control Your Robot!
        ---------------------------
        Moving around:
            W
        A    S    D

        Q : Stop the robot
        Esc : Quit
        """)

        twist = Twist()
        # Main loop to capture key presses and to publish velocity commands
        while True:
            key = self.get_key()
            
            if key == '\x1b':  # Escape key to quit
                break
            elif key == 'q':  # Stop the robot
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                self.get_logger().info("Robot Stopped")

            elif key == 'w':  # Forward
                self.linear_vel += LIN_VEL_STEP_SIZE
                self.get_logger().info("Robot Moving Forward")

            elif key == 's':  # Reverse
                self.linear_vel -= LIN_VEL_STEP_SIZE
                self.get_logger().info("Robot Moving Backward")
            elif key == 'a':  # Left turn
                self.angular_vel += ANG_VEL_STEP_SIZE
                self.get_logger().info("Robot Turning Left")

            elif key == 'd':  # Right turn
                self.angular_vel -= ANG_VEL_STEP_SIZE
                self.get_logger().info("Robot Turning Right")
            # Constrain steering range
            self.angular_vel = max(-1.0, min(self.angular_vel, 1.0))

            # Log velocity values
            self.get_logger().info(f"Linear Velocity: {self.linear_vel}, Angular Velocity: {self.angular_vel}")

            # Publish the Twist message
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel
            self.publisher.publish(twist)

def main():
    rclpy.init()
    node = Teleop()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
