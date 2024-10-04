import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
import matplotlib.pyplot as plt
import time

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, QoSProfile(depth=10))
        
        self.is_moving = False
        self.move_duration = 15.0  # Move for 15 seconds
        self.start_time = None  # Store the start time
        self.last_pose = None  # Store last pose
        self.distance_history = []  # For distance plotting
        self.time_history = []  # For time plotting
        self.pose_x_history = []  # For x position history
        self.pose_y_history = []  # For y position history
        self.timer = self.create_timer(0.1, self.update)  # Timer to update movement

    def odom_callback(self, msg):
        # Extract position from the odometry message
        position = msg.pose.pose.position
        self.last_pose = position  # Update the last pose
        
        # Record the x and y positions for plotting
        self.pose_x_history.append(position.x)
        self.pose_y_history.append(position.y)

    def update(self):
        if self.is_moving:
            current_time = time.time()
            if current_time - self.start_time < self.move_duration:
                msg = Twist()
                msg.linear.x = 0.1  # Maximum velocity value
                self.publisher.publish(msg)
                self.get_logger().info('Moving forward')

                # Record distance and time
                distance = self.calculate_distance()
                self.distance_history.append(distance)
                self.time_history.append(current_time - self.start_time)
            else:
                self.stop()  # Stop robot

    def calculate_distance(self):
        if self.last_pose:
            # Calculate distance from the origin (0,0)
            return math.sqrt(self.last_pose.x ** 2 + self.last_pose.y ** 2)
        return 0.0

    def start(self):
        self.is_moving = True
        self.start_time = time.time()  # Record start time
        self.distance_history.clear()  # Clear previous distance history
        self.time_history.clear()  # Clear previous time history
        self.pose_x_history.clear()  # Clear previous x history
        self.pose_y_history.clear()  # Clear previous y history
        self.get_logger().info('Starting movement.')

    def stop(self):
        msg = Twist()  # Stop robot
        self.publisher.publish(msg)
        self.get_logger().info('Robot has stopped.')
        self.is_moving = False
        self.timer.cancel()  # Stop timer

    def plot_pose(self):
        if self.distance_history and self.time_history:
            plt.figure()
            plt.plot(self.time_history, self.distance_history, marker='o')
            plt.title('Distance vs Time')
            plt.xlabel('Time (s)')
            plt.ylabel('Distance (m)')
            plt.grid()
            plt.axis('equal')

    def plot_trajectory(self):
        if self.pose_x_history and self.pose_y_history:
            plt.figure()
            plt.plot(self.pose_x_history, self.pose_y_history, marker='o')
            plt.title('Trajectory (X vs Y)')
            plt.xlabel('X Position (m)')
            plt.ylabel('Y Position (m)')
            plt.grid()
            plt.axis('equal')

def main(args=None):
    rclpy.init(args=args)
    controller = OpenLoopController()
    controller.start()
    
    while controller.is_moving:  # Spin while moving
        rclpy.spin_once(controller)  # Keep processing callbacks
    
    rclpy.shutdown()  # Shutdown node

    # Plot both graphs after shutting down the node
    controller.plot_pose()
    controller.plot_trajectory()
    
    plt.show()  # Show the plots

if __name__ == '__main__':
    main()
