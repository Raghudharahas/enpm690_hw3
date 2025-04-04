# Import necessary ROS2 message types
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#creating a new class called obstacle avoidance
#this class will inherit from the Node class
#this class will have a publisher and a subscriber
class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Declaration of  a tunable speed parameter
        self.declare_parameter('speed', 0.2)

        # Creating  publisher and subscriber
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10) # Subscribe to the Lidar data

        self.get_logger().info("Obstacle Avoidance Node Running...") # Print message to console

    #callback function that will be called when the subscriber receives a message
    def scan_callback(self, msg):
        twist = Twist()

        # Getting tunable speed parameter
        speed = self.get_parameter('speed').value

        # Getting  Lidar data and check for indices
        num_ranges = len(msg.ranges) # Number of Lidar readings
        # If no data received, print error message
        if num_ranges == 0:
            self.get_logger().error(" No Lidar data received!")
            return

        center_index = num_ranges // 2  # Center of the Lidar scan
        angle_range = int(num_ranges * (30 / 360))  # Convert 30° to array indices

        # Extracting only the front Lidar readings (±15° from center)
        front_ranges = msg.ranges[center_index - angle_range:center_index + angle_range]

        # Printing the Lidar data for reference 
        self.get_logger().info(f" Front Lidar Data: {front_ranges[:5]} ... {front_ranges[-5:]}")

        # Filter invalid values (remove 'inf' readings)
        valid_ranges = [r for r in front_ranges if 0.01 < r < float('inf')]

        # If all values are  inf or empty, assume no obstacles
        min_distance = min(valid_ranges) if valid_ranges else 2.0

        # Print the closest obstacle distance
        self.get_logger().info(f" Closest Obstacle Distance (Front 30°): {min_distance:.2f} meters")

        # Check if obstacle is too close and take action
        if min_distance < 0.5:
            self.get_logger().info(" Obstacle Detected! Turning Left...")
            twist.angular.z = 0.6  # Turn left
            twist.linear.x = 0.0   # Stop forward motion
        else:
            self.get_logger().info(" Path Clear! Moving Forward...")
            twist.linear.x = -speed  # Move forward
            twist.angular.z = 0.0   # Go straight

        # Publish movement command
        self.publisher.publish(twist)

        
#main function that will create an instance of the ObstacleAvoidance class
def main():
    rclpy.init()
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
