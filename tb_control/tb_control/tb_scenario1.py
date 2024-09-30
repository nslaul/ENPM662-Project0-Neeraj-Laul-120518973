import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.is_moving = False
        self.move_duration = 15 # Total time to move in seconds
        self.elapsed_time = 0.0   # Time elapsed since start
        self.timer = self.create_timer(0.1, self.update)  # Timer to update movement

    def update(self):
        if self.is_moving:
            if self.elapsed_time < self.move_duration:
                msg = Twist()
                msg.linear.x = 0.1  # Maximuim velocity value
                self.publisher.publish(msg)
                self.elapsed_time += 0.1
                self.get_logger().info('Moving forward')
            else:
                self.stop()  # Stop robot

    def start(self):
        self.is_moving = True
        self.get_logger().info('Starting movement.')

    def stop(self):
        msg = Twist()  # Stop robot
        self.publisher.publish(msg)
        self.get_logger().info('Robot has stopped.')
        self.is_moving = False
        self.timer.cancel()  # Stop timer

def main(args=None):
    rclpy.init(args=args)
    controller = OpenLoopController()
    controller.start()
    
    while controller.is_moving:  # Spin while moving
        rclpy.spin_once(controller)  # Keep processing callbacks
    
    rclpy.shutdown()  # Shutdown node for ease of terminal control, dont have to keep pressing ctrlC

if __name__ == '__main__':
    main()

