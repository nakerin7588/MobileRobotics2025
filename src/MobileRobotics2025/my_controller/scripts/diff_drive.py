#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from math import cos, sin

from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class DiffDriveKinematics:
    def __init__(self):

        self.wheel_radius = 0.05 # meters
        self.wheel_base = 0.175 * 2.0 # meters

        self.wheel_speed_l = 0.0 # wheel speed left
        self.wheel_speed_r = 0.0 # wheel speed right
        
        self.robot_speed_v = 0.0 # robot speed linear
        self.robot_speed_w = 0.0 # robot speed angular

        self.x = 0.0
        self.y = 0.0

        self.theta = 0.0

        self.dt = 0.01

    def forward_kinematics(self, w_l, w_r):
        
        self.robot_speed_v = self.wheel_radius * (w_l + w_r) / 2.0
        self.robot_speed_w = self.wheel_radius * (w_r - w_l) / self.wheel_base

        return self.robot_speed_v, self.robot_speed_w

    def inverse_kinematics(self, vx, wz):

        self.wheel_speed_l = self.wheel_radius * ((vx/self.wheel_radius) - (self.wheel_base / (2.0 * self.wheel_radius)) * wz)
        self.wheel_speed_r = self.wheel_radius * ((vx/self.wheel_radius) + (self.wheel_base / (2.0 * self.wheel_radius)) * wz)

        return self.wheel_speed_l, self.wheel_speed_r

    def odometry(self):

        self.x += self.robot_speed_v * cos(self.theta) * self.dt
        self.y += self.robot_speed_v * sin(self.theta) * self.dt
        self.theta += self.robot_speed_w * self.dt

        return self.x, self.y, self.theta


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        rate = 100 # Hz

        self.timer = self.create_timer(1/rate, self.timer_callback)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback,10)

        self.vel_controllers_commands_publisher = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands',10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom',10)

        self.diff_drive_kinematics = DiffDriveKinematics()

        self.diff_drive_kinematics.dt = 1.0 / rate

        self.tf_broadcaster = TransformBroadcaster(self)

    def cmd_vel_callback(self, msg: Twist):
        self.diff_drive_kinematics.inverse_kinematics(msg.linear.x, msg.angular.z)
        self.get_logger().info('Wheel Speed Left: %f, Wheel Speed Right: %f' % (self.diff_drive_kinematics.wheel_speed_l, self.diff_drive_kinematics.wheel_speed_r))

    def timer_callback(self):
        vel_msg = Float64MultiArray()
        vel_msg.data = [self.diff_drive_kinematics.wheel_speed_l, self.diff_drive_kinematics.wheel_speed_r]
        self.vel_controllers_commands_publisher.publish(vel_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, theta = self.diff_drive_kinematics.odometry()

        q = quaternion_from_euler(0.0, 0.0, theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.diff_drive_kinematics.robot_speed_v
        odom_msg.twist.twist.angular.z = self.diff_drive_kinematics.robot_speed_w

        self.odom_publisher.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
