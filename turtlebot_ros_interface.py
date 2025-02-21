#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import WheelTicks, IrIntensityVector
from drive import Drive
import time
from rclpy.qos import qos_profile_sensor_data
from robot import Robot
import numpy as np

TIME_STEP = 0.03125


class Point:
    def __init__(self, x, y):
        """
        Represents a 2D point with x and y coordinates.
        """
        self.x = x
        self.y = y


class TurtleBot(Node):
    """
    A ROS2 node acting as an interface for TurtleBot robot. This class handles sensor
    data and transforms to suitable format.
    """
    def __init__(self):
        super().__init__('turtlebot')
        self.drive_node = Drive()
        self.timer = self.create_timer(TIME_STEP, self.run)

        # Initialize sensor and state variables
        self.left_wheel_encoder = 0.0
        self.right_wheel_encoder = 0.0
        self.lidar_ranges = []
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.ir_intensity_values = {}
        self.WHEEL_BASE = 0.233
        self.WHEEL_RADIUS = 0.03575
        self.WHEEL_DIAMETER = self.WHEEL_RADIUS * 2
        self.ir_intensity_values = {
            'ir_intensity_side_left': 0,
            'ir_intensity_left': 0,
            'ir_intensity_front_left': 0,
            'ir_intensity_front_center_left': 0,
            'ir_intensity_front_center_right': 0,
            'ir_intensity_front_right': 0,
            'ir_intensity_right': 0,
        }

        # Camera parameters
        self.IMAGE_HEIGHT = 750
        self.IMAGE_WIDTH = 750
        self.image_HFOV = 0.785  # radians
        self.image_data = None

        # Sensor subscriptions
        self.create_subscription(IrIntensityVector, '/tb4_33/ir_intensity',
                                 self.ir_intensity_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/tb4_33/odom', self.odom_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/tb4_33/scan', self.scan_callback,
                                 qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(WheelTicks, '/tb4_33/wheel_ticks',
                                 self.wheel_ticks_callback, 10)
        self.create_subscription(CameraInfo, '/tb4_33/oakd/rgb/preview/camera_info',
                                 self.camera_info_callback,
                                 qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Image, '/tb4_33/oakd/rgb/preview/image_raw',
                                 self.camera_image_callback,
                                 qos_profile=rclpy.qos.qos_profile_sensor_data)

    @staticmethod
    def sleep(duration):
        """Sleep to maintain necessary frequency."""
        time.sleep(duration)

    def run(self):
        """Periodically publish drive commands."""
        self.drive_node.publish_speeds()

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera parameters from CameraInfo message."""
        self.image_width = msg.width
        self.image_height = msg.height

    def camera_image_callback(self, msg: Image):
        """Process raw image data from the camera."""
        rgb_data = list(msg.data)
        self.image_data = np.array(rgb_data, dtype=np.uint8).reshape(
            (self.image_height, self.image_width, 3))

    def get_orientation(self):
        """Return the robot's current yaw (orientation)."""
        return self.odom_yaw

    def get_time(self) -> float:
        """Return the current system time."""
        return time.time()

    def get_simulator(self) -> bool:
        """Return whether running in simulator or on real robot.

        Returns:
            True if simulation, False if real robot.
        """
        return False

    def get_realistic(self) -> bool:
        """Return whether running in realistic mode or not.

        Returns:
            True if realistic, False if not.
        """
        return True

    def get_lidar_range_list(self) -> list:
        """Return the processed list of LiDAR range readings."""
        return self.lidar_ranges

    def get_lidar_point_cloud(self) -> list:
        """Return the processed list of LiDAR point cloud readings."""
        pointcloud = []
        if self.lidar_ranges != []:
            range_list = [None if math.isinf(value) else value for value in
                          self.lidar_ranges]
            lidar_angles = np.linspace(0.0 - np.pi / 2,
                                       0.0 - np.pi / 2 - 2 * np.pi,
                                       len(range_list), endpoint=False)
            angles = [((angle + np.pi) % (2 * np.pi)) - np.pi for angle in
                      lidar_angles]
            i = 0
            for point in range_list:
                if point is None:
                    i += 1
                    pointcloud.append(Point(float('inf'), float('inf')))
                else:
                    x_robot = (point * math.sin(angles[i]))
                    y_robot = (-point * math.cos(angles[i]))
                    pointcloud.append(Point(x_robot, y_robot))
                    i += 1
            return pointcloud
        return None

    def set_left_motor_torque(self, torque: float) -> None:
        """Adjust the torque for the left motor to control its speed and direction.

        This method applies a specified torque to the left motor, influencing both
        its speed and direction. Positive torque results in forward movement,
        negative torque results in backward movement, and zero torque stops the motor.

        Args:
            torque: The torque value in Newton-meters (Nm), where:
                    - Positive values (> 0 Nm) result in forward movement.
                    - Negative values (< 0 Nm) result in backward movement.
                    - 0 Nm stops the motor.

        """
        self.drive_node.set_left_wheel_speed(torque)

    def set_right_motor_torque(self, torque: float) -> None:
        """Adjust the torque for the right motor to control its speed and direction.

        This method applies a specified torque to the right motor, influencing both
        its speed and direction. Positive torque results in forward movement,
        negative torque results in backward movement, and zero torque stops the motor.

        Args:
            torque: The torque value in Newton-meters (Nm), where:
                    - Positive values (> 0 Nm) result in forward movement.
                    - Negative values (< 0 Nm) result in backward movement.
                    - 0 Nm stops the motor.

        """
        self.drive_node.set_right_wheel_speed(torque)

    def get_camera_rgb_image(self):
        """Return an image from the camera as a NumPy array in BGRA format.

        This method retrieves the raw image data from the camera's RGB sensor,
        converts it into a NumPy array, and reshapes it into the correct
        dimensions corresponding to the height, width, and number of channels
        (BGRA). The image data is returned in the format (height, width, 4),
        where the last dimension represents the BGRA channels (Blue, Green,
        Red, Alpha).

        Returns:
            A 3D NumPy array of shape (height, width, 4) representing the
            captured BGRA image, with pixel values as unsigned integers.

        """
        return self.image_data

    def get_camera_params(self):
        """Returns the camera HFOV (Horizontal Field of View).

        Returns:
            FOV in radians.

        """
        return self.image_HFOV

    def get_camera_depth_image(self):
        """Return the depth image data from the camera.

        Returns:
            A 2D NumPy array of shape (height, width) representing the captured
            depth image, with pixel values as float values.

        """
        return None

    def set_left_motor_velocity(self, wheel_speed):
        """Set the rotational velocity of the left motor.

        Args:
            speed: The desired rotational velocity for the right motor in radians per
                   second. Positive values indicate forward rotation, while negative
                   values indicate backward rotation.

        """
        self.drive_node.set_left_wheel_speed(wheel_speed)

    def set_right_motor_velocity(self, wheel_speed):
        """Set the rotational velocity of the right motor.

        Args:
            speed: The desired rotational velocity for the right motor in radians per
                   second. Positive values indicate forward rotation, while negative
                   values indicate backward rotation.

        """
        self.drive_node.set_right_wheel_speed(wheel_speed)

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion orientation to yaw angle."""
        x, y, z, w = quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Update odometry information from the Odometry message."""
        pose = msg.pose.pose
        x_position = pose.position.x
        y_position = pose.position.y
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = self.quaternion_to_yaw(quaternion)
        self.odom_yaw = yaw
        self.odom_x = x_position
        self.odom_y = y_position

    def get_odom(self):
        """Return the current odometry data (x, y, yaw)."""
        return [self.odom_x, self.odom_y, self.odom_yaw]

    def get_ir_intensities_list(self) -> list:
        """Return the infra-red sensors intensities as a list.

        Returns:
            The returned list has the following order [left, side_left,
            center_left, center, center_right, side_right, right].

        """
        return [
            self.ir_intensity_values.get('ir_intensity_side_left', None),
            self.ir_intensity_values.get('ir_intensity_left', None),
            self.ir_intensity_values.get('ir_intensity_front_left', None),
            self.ir_intensity_values.get('ir_intensity_front_center_left', None),
            self.ir_intensity_values.get('ir_intensity_front_center_right', None),
            self.ir_intensity_values.get('ir_intensity_front_right', None),
            self.ir_intensity_values.get('ir_intensity_right', None)]

    def get_ir_intensity_left(self) -> float:
        """Return the leftmost infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_left', None)

    def get_ir_intensity_side_left(self) -> float:
        """Return the second leftmost infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_side_left', None)

    def get_ir_intensity_center_left(self) -> float:
        """Return the third leftmost infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_front_left', None)

    def get_ir_intensity_center(self) -> float:
        """Return the center infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_front_center_left', None)

    def get_ir_intensity_center_right(self) -> float:
        """Return the third rightmost infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_front_center_right', None)

    def get_ir_intensity_side_right(self) -> float:
        """Return the second rightmost infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_front_right', None)

    def get_ir_intensity_right(self) -> float:
        """Return the rightmost infra-red sensor value.

        Returns:
            The distance value as a float.
            The value range is from 0 (far) to 4000 (close).

        """
        return self.ir_intensity_values.get('ir_intensity_right', None)

    def ir_intensity_callback(self, msg: IrIntensityVector):
        """Update IR intensity values from sensor readings."""
        for reading in msg.readings:
            frame_id = reading.header.frame_id
            value = reading.value
            self.ir_intensity_values[frame_id] = value

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan data and extract useful information."""
        filtered_ranges = []
        interval = 1080 / 640
        for i in range(640):
            index = int(i * interval)
            filtered_ranges.append(msg.ranges[index])
        final_ranges = filtered_ranges
        final_ranges.reverse()
        self.lidar_ranges = final_ranges

    def get_left_motor_encoder_ticks(self) -> int:
        """Return the number of ticks from the right motor encoder.

        Based on documentation at: https://iroboteducation.github.io/create3_docs/api/odometry/

        Returns:
            The number of ticks as an integer.

        """
        return round(self.left_wheel_encoder)

    def get_right_motor_encoder_ticks(self) -> int:
        """Return the number of ticks from the right motor encoder.

        Based on documentation at: https://iroboteducation.github.io/create3_docs/api/odometry/

        Returns:
            The number of ticks as an integer.

        """
        return round(self.right_wheel_encoder)

    def wheel_ticks_callback(self, msg: WheelTicks):
        """Update wheel encoder tick counts."""
        self.left_wheel_encoder = msg.ticks_left
        self.right_wheel_encoder = msg.ticks_right

    def spin(self):
        """Spin the ROS2 node once."""
        rclpy.spin_once(self)

    def stop(self):
        """Shutdown the TurtleBot node."""
        turtlebot.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Main function to initialize and run the TurtleBot node."""
    rclpy.init(args=args)
    turtlebot = TurtleBot()
    robot = Robot(turtlebot)
    try:
        while True:
            rclpy.spin_once(turtlebot)
            robot.spin()
            time.sleep(TIME_STEP)
    except KeyboardInterrupt:
        print("stopping robot")
    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

