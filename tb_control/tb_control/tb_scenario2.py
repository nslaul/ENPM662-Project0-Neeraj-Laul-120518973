import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.is_moving = False
        self.current_velocity = 0.0 
        self.max_velocity = 0.1  # Max velocity
        self.acceleration = 0.02  # Acceleration
        self.start_time = None

    def publish_velocity(self):
        if self.is_moving:
            elapsed_time = time.time() - self.start_time
            if elapsed_time < 5:  # Accelerate for 5 seconds
                self.current_velocity += self.acceleration
                if self.current_velocity > self.max_velocity:
                    self.current_velocity = self.max_velocity
            elif elapsed_time < 10:  # Constant velocity for 5 seconds
                pass
            else:  # Decelerate
                self.current_velocity -= self.acceleration
                if self.current_velocity < 0:
                    self.current_velocity = 0.0  

            # Create and publish the message
            msg = Twist()
            msg.linear.x = float(self.current_velocity)  
            self.publisher.publish(msg)
            self.get_logger().info(f'Moving with velocity: {self.current_velocity}')

            if elapsed_time >= 15:  # Stop after total 15 seconds
                self.is_moving = False
                self.timer.cancel()  # Stop the timer
                self.get_logger().info('Robot has stopped.')

    def start(self):
        self.is_moving = True
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.publish_velocity)

def main(args=None):
    rclpy.init(args=args)
    controller = OpenLoopController()
    controller.start()
    while controller.is_moving:  
        rclpy.spin_once(controller)
    
    rclpy.shutdown()  # Shutdown after the movement ends. Easier than always pressing ctrlC

if __name__ == '__main__':
    main()

