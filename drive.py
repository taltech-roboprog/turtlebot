#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Drive(Node):
    """
    A ROS2 node for handling motor control of a differential-drive robot.
    This class publishes velocity commands to the robot's movement topic.
    """
    def __init__(self):
        super().__init__('drive')
        self.publisher = self.create_publisher(Twist, '/tb4_33/cmd_vel', 10)
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.max_speed = 0.2

    def publish_speeds(self):
        """
        Compute the linear and angular velocities based on wheel speeds
        and publish them as a Twist message.
        """
        self.linear_vel = (self.left_wheel_speed + self.right_wheel_speed) / 2.0
        self.angular_vel = (self.right_wheel_speed - self.left_wheel_speed) / 0.233
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.publisher.publish(msg)

    def set_speeds(self, left_wheel_speed, right_wheel_speed):
        """Set the speed of both wheels, ensuring they stay within the max limit."""
        self.left_wheel_speed = self.limit_speed(left_wheel_speed)
        self.right_wheel_speed = self.limit_speed(right_wheel_speed)

    def set_left_wheel_speed(self, speed):
        """Set the speed of the left wheel, ensuring it stays within the max limit."""
        self.left_wheel_speed = self.limit_speed(speed)

    def set_right_wheel_speed(self, speed):
        """Set the speed of the right wheel, ensuring it stays within the max limit."""
        self.right_wheel_speed = self.limit_speed(speed)

    def set_both_wheel_speeds(self, speed):
    	"""Set both wheels to the same speed, ensuring they stay within the max limit."""
        self.right_wheel_speed = self.limit_speed(speed)
        self.left_wheel_speed = self.limit_speed(speed)

    def limit_speed(self, speed):
        """Ensure the given speed does not exceed the maximum allowed value."""
        if speed > self.max_speed:
            return self.max_speed
        elif speed < -self.max_speed:
            return -self.max_speed
        return speed

def main(args=None):
    """Main function to initialize the ROS2 node and start the Drive node."""
    rclpy.init(args=args)
    drive = Drive()
    rclpy.spin(drive)
    drive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

