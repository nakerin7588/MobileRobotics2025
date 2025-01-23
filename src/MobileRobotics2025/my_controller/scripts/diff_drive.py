#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class DiffDriveKinematics:
    def __init__(self):

        self.wheel_radius = 0.05 # meters
        self.wheel_base = 0.175 * 2.0 # meters

        self.wheel_speed_l = 0.0 # wheel speed left
        self.wheel_speed_r = 0.0 # wheel speed right
        
        self.robot_speed_v = 0.0 # robot speed linear
        self.robot_speed_w = 0.0 # robot speed angular

    def forward_kinematics(self, w_l, w_r):
        
        self.robot_speed_v = self.wheel_radius * (w_l + w_r) / 2.0
        self.robot_speed_w = self.wheel_radius * (w_r - w_l) / self.wheel_base

        return self.robot_speed_v, self.robot_speed_w

    def inverse_kinematics(self, vx, wz):

        self.wheel_speed_l = self.wheel_radius * ((vx/self.wheel_radius) - (self.wheel_base / (2.0 * self.wheel_radius)) * wz)
        self.wheel_speed_r = self.wheel_radius * ((vx/self.wheel_radius) + (self.wheel_base / (2.0 * self.wheel_radius)) * wz)

        return self.wheel_speed_l, self.wheel_speed_r

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        rate = 100 # Hz

        self.timer = self.create_timer(1/rate, self.timer_callback)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback,10)

        self.vel_controllers_commands_publisher = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands',10)

        self.diff_drive_kinematics = DiffDriveKinematics()

    def cmd_vel_callback(self, msg: Twist):
        self.diff_drive_kinematics.inverse_kinematics(msg.linear.x, msg.angular.z)
        self.get_logger().info('Wheel Speed Left: %f, Wheel Speed Right: %f' % (self.diff_drive_kinematics.wheel_speed_l, self.diff_drive_kinematics.wheel_speed_r))

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.diff_drive_kinematics.wheel_speed_l, self.diff_drive_kinematics.wheel_speed_r]
        self.vel_controllers_commands_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
