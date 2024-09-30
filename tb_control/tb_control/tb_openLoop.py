import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('tb_openLoop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Parameters for motion
        self.distance = 1.0  # Distance in meters
        self.time_constant_velocity = 10.0  # Time in seconds for constant velocity
        self.current_time = 0.0
        
        # Scenario parameters
        self.acceleration = 0.1  # m/s^2
        self.max_velocity = self.distance / self.time_constant_velocity  # Constant velocity
        self.state = "acceleration"  # Initial state
        self.velocity = 0.0
        self.cmd = Twist()

    def timer_callback(self):
        if self.state == "acceleration":
            if self.velocity < self.max_velocity:
                self.velocity += self.acceleration * 0.1  # Increment velocity
                if self.velocity >= self.max_velocity:
                    self.velocity = self.max_velocity
                    self.state = "constant_velocity"  # Transition to constant velocity
            self.cmd.linear.x = self.velocity
        
        elif self.state == "constant_velocity":
            self.current_time += 0.1
            if self.current_time >= self.time_constant_velocity:
                self.state = "deceleration"  # Transition to deceleration
            self.cmd.linear.x = self.velocity
        
        elif self.state == "deceleration":
            self.velocity -= self.acceleration * 0.1  # Decrement velocity
            if self.velocity <= 0.0:
                self.velocity = 0.0
                self.cmd.linear.x = 0.0  # Stop the robot
                self.get_logger().info('Robot has stopped.')
                self.destroy_timer(self.timer)  # Stop the timer
            else:
                self.cmd.linear.x = self.velocity
        
        self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    open_loop_controller = OpenLoopController()
    rclpy.spin(open_loop_controller)
    open_loop_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

