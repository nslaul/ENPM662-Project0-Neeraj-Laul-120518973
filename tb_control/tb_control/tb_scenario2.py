import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import time
import math

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameters
        self.is_moving = False
        self.current_velocity = 0.0 
        self.max_velocity = 0.5  # Maximum velocity (m/s)
        self.acceleration = 0.02  # Acceleration (m/s^2)
        self.deceleration = 0.02  # Deceleration (m/s^2)
        self.target_distance = 2.0  # Total distance to travel (m)
        self.distance_traveled = 0.0  # Total distance traveled
        self.state = 'accelerating'  # Current state of the robot

        # For plotting
        self.time_history = []  # Track time elapsed
        self.distance_history = []  # Track distance traveled
        self.pose_x_history = []  # For x position history
        self.pose_y_history = []  # For y position history
        self.start_time = None  # Record the start time

    def publish_velocity(self):
        if self.is_moving:
            if self.state == 'accelerating':
                # Accelerate until reaching maximum velocity
                if self.current_velocity < self.max_velocity:
                    self.current_velocity += self.acceleration
                    if self.current_velocity > self.max_velocity:
                        self.current_velocity = self.max_velocity

                # Transition to constant velocity if target distance is reached
                if self.distance_traveled >= self.target_distance:
                    self.state = 'decelerating'

            elif self.state == 'decelerating':
                # Calculate stop distance
                stop_distance = (self.current_velocity ** 2) / (2 * self.deceleration)

                # Check if enough distance remains to decelerate
                if self.distance_traveled + stop_distance >= self.target_distance:
                    # Start deceleration
                    if self.current_velocity > 0:
                        self.current_velocity -= self.deceleration
                        if self.current_velocity < 0:
                            self.current_velocity = 0.0

                # If we've stopped, mark the movement as complete
                if self.current_velocity == 0:
                    self.is_moving = False
                    self.timer.cancel()  # Stop the timer
                    self.get_logger().info('Robot has stopped.')

            # Create and publish the message
            msg = Twist()
            msg.linear.x = self.current_velocity  
            self.publisher.publish(msg)
            self.get_logger().info(f'Moving with velocity: {self.current_velocity:.2f}')

            # Update total distance traveled
            self.distance_traveled += self.current_velocity * 0.1  # Assuming the timer interval is 0.1 seconds
            elapsed_time = time.time() - self.start_time  # Calculate elapsed time
            self.time_history.append(elapsed_time)  # Store elapsed time
            self.distance_history.append(self.distance_traveled)  # Store distance traveled

            # Update pose history (assuming movement along the x-axis)
            self.pose_x_history.append(self.distance_traveled)
            self.pose_y_history.append(0)  # Assume y position remains 0 for straight line

    def start(self):
        self.is_moving = True
        self.distance_traveled = 0.0  # Reset total distance
        self.current_velocity = 0.0  # Reset velocity
        self.state = 'accelerating'  # Start with acceleration
        self.start_time = time.time()  # Record the start time
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def plot_graph(self):
        plt.figure()
        plt.plot(self.time_history, self.distance_history, marker='o')
        plt.title('Distance vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Distance (m)')
        plt.grid()
        plt.axis('equal')

    def plot_trajectory(self):
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
    
    while controller.is_moving:  
        rclpy.spin_once(controller)

    rclpy.shutdown()  # Shutdown after the movement ends

    # Plot both graphs after shutting down the node
    controller.plot_graph()
    controller.plot_trajectory()
    
    plt.show()  # Show the plots

if __name__ == '__main__':
    main()
