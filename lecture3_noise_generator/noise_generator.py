#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
import sys
from lecture3_interfaces.srv import SetNoise


# ros2 topic hz /noise


class NoiseGenerator(Node):
    def __init__(self):
        super().__init__('noise_generator_node')
        self.declare_parameter('rate', 5.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.noise_pub = self.create_publisher(Float64, 'noise', 10)
        self.mean = 0.0
        self.variance = 1.0
        self.set_noise_server = self.create_service(SetNoise, 'set_noise', self.set_noise_callback)
        self.timer = self.create_timer((1/self.rate), self.timer_callback)
        self.get_logger().info(f'Starting {self.get_namespace()} {self.get_name()}')


    def set_noise_callback(self, req : SetNoise.Request, resp : SetNoise.Request):
        self.mean = req.mean.data
        self.variance = req.variance.data
        return resp
    

    def timer_callback(self):
        msg = Float64()
        msg.data = np.random.normal(self.mean, np.sqrt(self.variance))
        self.noise_pub.publish(msg)
    

    

def main(args=None):
    rclpy.init(args=args)
    node = NoiseGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()