#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.yaw_vel = 0.
        self.L = 1
        self.ittc_thresh = 2
        self.prev_ts = None

        # Subscribe to the scan topic to receive laser readings
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10
        )

        # Subscribe to the odometry topic to get odometry readings
        self.odom_sub = self.create_subscription(
            Odometry,
            "ego_racecar/odom",
            self.odom_callback,
            10
        )

        # Create a publisher to publish control commands to the car.
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "drive", 10)


    def odom_callback(self, odom_msg : Odometry):
        # Look a the odometry and look at the velocity of the car
        # This is telling us simple where the vehicle is heading
        self.speed = odom_msg.twist.twist.linear.x
        self.yaw_vel = odom_msg.twist.twist.angular.z
        # if odom_msg.twist.twist.linear.y != 0:
        #     print(odom_msg.twist.twist.linear.y)

    def scan_callback(self, scan_msg : LaserScan):
        # These are the angles between the beams and the body frame
        thetas = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        ranges = scan_msg.ranges

        # Set timestamps for projection
        # last = scan_msg.header.stamp.sec + (scan_msg.header.stamp.nanosec/1e+9)
        # self.current_time = self.get_clock().now().to_msg()
        # curr = self.current_time.sec + (self.current_time.nanosec/1e+9)
        # dt = curr - last

        # TODO: Extra credit with bearing
        # The bearing plus the laser angles forms
        
        # bear_adj_laser_angles = laser_angles + self.bearing

        # Vehicle velocity projected onto the lasers
        if self.speed is not None:
            if self.yaw_vel == 0:
                beta = 0
                vx = self.speed
                vy = 0
            else:
                # Find the steering angle, use that to calculate the velocity bearing
                # Ratio of rate of turn (and direction) divided by the speed
                # yaw_vel takes into account the direction of the turning
                theta_f = np.arctan2(2 * self.yaw_vel, self.speed) 
                beta = np.arctan2(np.tan(theta_f), 2)

                # Velocity along the x direction of the body frame
                # Should be 0 when driving straight
                vx = self.speed * np.cos(beta)
                vy = self.speed * np.sin(beta)
                # print(vy)

            # Look ahead prediction based on the velocity L seconds in the future
            proj_vel_long = vx * np.cos(thetas)
            proj_ranges = ranges - proj_vel_long * self.L
            r_dot = proj_ranges - ranges # Positive if we are moving further away, negative if we are getting closer
            iTTC = ranges / np.clip(-r_dot, a_min=0, a_max=np.inf)

            # Increase the lasers we care about when turning
            laser_radius = beta
            num_entries = int(laser_radius / scan_msg.angle_increment)
            middle_idx = len(iTTC)//2

            # Find the low and high idxs
            lo = middle_idx
            hi = middle_idx + 1
            if vy < 0:
                lo = max(0, middle_idx - num_entries)
            elif vy > 0:
                hi = max(len(iTTC), middle_idx + num_entries + 1)
            selected_iTTC = iTTC[lo:hi]
            if np.any(selected_iTTC < self.ittc_thresh):
                print("STOP!")
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = self.get_clock().now().to_msg()
                drive_msg.drive.speed = 0.
                self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()