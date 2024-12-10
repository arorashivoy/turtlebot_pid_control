import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math


class TurtleBotController(Node):
    def __init__(self, T, dt, targets):
        super().__init__('turtlebot_controller')

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time and control parameters
        self.t = 0.0
        self.T = T
        self.dt = dt

        # Targets
        self.target_index = 0
        self.targets = targets

        # Velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Odom subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # State tracking
        self.x_points = []
        self.y_points = []

    def odom_callback(self, msg):
        """Callback to update robot pose from /odom."""
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y

        # Convert quaternion to yaw (theta)
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def controller(self):
        """Proportional controller to compute linear and angular velocities."""
        r = np.sqrt((self.x)**2 + (self.y)**2)

        x_target = self.targets[self.target_index][0]
        y_target = self.targets[self.target_index][1]

        k_w = 3
        k_v = 0.5
        # _theta = _theta if _theta >= 0 else 2 * np.pi + _theta
        error_w = np.arctan2(y_target - self.y, x_target - self.x) - self.theta
        while error_w < -np.pi:
            error_w += 2 * np.pi
        _w = k_w * error_w

        error_v = np.sqrt((x_target - self.x)**2 + (y_target - self.y)**2)
        _v = k_v * error_v

        _d = abs(_v) + abs(_w * r)

        # v = _v / _d * 2
        # w = _w / _d * 2

        v = 0.5
        w = _w
        # v = 2.0

        # Check if close to the target
        if (abs(self.x - self.targets[self.target_index][0]) < 0.05 and
                abs(self.y - self.targets[self.target_index][1]) < 0.05):
            self.get_logger().info(f"reached target: {self.targets[self.target_index]}")
            self.target_index += 1
            self.target_index %= len(self.targets)
            # w = 0.0
            v = 2.0

            self.get_logger().info(f"Going to next target: {self.targets[self.target_index]}")

        # Publish the velocity
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_vel_pub.publish(cmd)

        # Log and save points for plotting
        self.x_points.append(self.x)
        self.y_points.append(self.y)

    def timer_callback(self):
        """Main control loop."""
        if self.t < self.T:
            self.controller()
            self.t += self.dt
        else:
            # Stop the robot when the time ends
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Finished!')

    def plot(self):
        """Optional: Plot trajectory using matplotlib."""
        import matplotlib.pyplot as plt
        plt.plot(self.x_points, self.y_points, label="Path")
        plt.scatter(self.targets[:, 0], self.targets[:, 1], color="red", label="Targets")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.title("TurtleBot Trajectory")
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    # Define target locations
    targets = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])

    # Run controller
    controller = TurtleBotController(T=25.0, dt=0.005, targets=targets)
    rclpy.spin(controller)

    # Optional: Plot trajectory
    controller.plot()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
