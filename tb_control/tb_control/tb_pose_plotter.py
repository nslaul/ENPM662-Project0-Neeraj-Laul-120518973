import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class RealTimePosePlotter(Node):
    def __init__(self):
        super().__init__('real_time_pose_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # This is the typical odometry topic for TurtleBot
            self.pose_callback,
            10
        )
        
        self.poses = []
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('TurtleBot Pose in Real Time')
        self.line, = self.ax.plot([], [], 'b-')  # Initialize line object
        self.ax.set_xlim(-3, 3)  # Adjust based on your environment
        self.ax.set_ylim(-3, 3)  # Adjust based on your environment
        plt.ion()  # Turn on interactive mode
        plt.show()

    def pose_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Append pose data
        self.poses.append((x, y))
        
        # Update plot
        self.update_plot()

    def update_plot(self):
        if self.poses:
            x_vals, y_vals = zip(*self.poses)
            self.line.set_xdata(x_vals)
            self.line.set_ydata(y_vals)
            self.ax.relim()  # Recalculate limits
            self.ax.autoscale_view()  # Rescale the view
            plt.draw()  # Redraw the plot
            plt.pause(0.01)  # Pause to allow the plot to update

def main(args=None):
    rclpy.init(args=args)
    plotter = RealTimePosePlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
