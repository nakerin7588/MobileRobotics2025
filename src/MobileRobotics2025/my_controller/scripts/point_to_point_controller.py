#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from math import atan2, sin, cos

class PointToPointControllerNode(Node):
    def __init__(self):
        super().__init__('point_to_point_controller_node')

        rate = 100 # Hz
        self.create_timer(1/rate, self.timer_callback)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_state = "IDLE"

        self.goal_x = 0.0 
        self.goal_y = 0.0 
        self.goal_theta = 0.0

        self.robot_x = 0.0 # current x
        self.robot_y = 0.0 # current y
        self.robot_theta = 0.0 # current theta

    def timer_callback(self):
        if self.robot_state == "RUNNING" and (self.robot_x != self.goal_x or self.robot_y != self.goal_y or self.robot_theta != self.goal_theta):
            d = ((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)**0.5
            e_theta = atan2(sin(self.goal_theta - self.robot_theta), cos(self.goal_theta - self.robot_theta))

            if d < 0.1 and abs(e_theta) < 0.1:
                self.robot_state = "IDLE"
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(msg)
                return

            vx = 0.5 * d
            wz = 4.0 * e_theta
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            self.cmd_vel_publisher.publish(msg)

    def odometry_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = msg.pose.pose.orientation.z

    def goal_pose_callback(self, msg: PoseStamped):
        if self.robot_state == "IDLE":
            self.robot_state = "RUNNING"
            self.get_logger().info('Goal Pose Received: %f, %f' % (msg.pose.position.x, msg.pose.position.y))
            self.goal_x = msg.pose.position.x
            self.goal_y = msg.pose.position.y
            self.goal_theta = msg.pose.orientation.z
        else:
            self.get_logger().info('Robot is already running, wait until it reaches the goal pose')
    
def main(args=None):
    rclpy.init(args=args)
    node = PointToPointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
