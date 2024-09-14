#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
import sys
from geometry_msgs.msg import Twist





class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux_node')
        self.declare_parameter('rate', 5.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.linear_vel_sub = self.create_subscription(Float64, '/linear/noise', self.linear_vel_callback, 10)
        self.angular_vel_sub = self.create_subscription(Float64, '/angular/noise', self.angular_vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()
        self.create_timer((1/self.rate), self.timer_callback)
        self.get_logger().info(f'Starting {self.get_namespace()} {self.get_name()}')

  
    def linear_vel_callback(self, msg : Float64):
        self.cmd_vel.linear.x = msg.data

    def angular_vel_callback(self, msg : Float64):
        self.cmd_vel.angular.z = msg.data

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.cmd_vel)

    

def main(args=None):
    rclpy.init(args=args)
    node = VelocityMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()