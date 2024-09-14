#!/usr/bin/python3
import math
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, TransformStamped
from turtlesim.msg import Pose
import numpy as np
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        self.__rate = 100
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.mouse_position_sub = self.create_subscription(Point, '/mouse_position', self.mouse_position_calback, 10)
        self.spawn_pizza_cl = self.create_client(GivePosition, 'spawn_pizza')
        self.eat_pizza_cl = self.create_client(Empty, 'turtle1/eat')
        self.create_timer(1/self.__rate, self.timer_callback)
        self.goal_positions = []
        self.set_position()


    def set_position(self):
        self.robot_pose = np.array([0.0, 0.0, 0.0]) # x, y, theta
        self.mouse_position = np.array([0.0, 0.0]) # x, y


    def eat_pizza(self):
        eat_req = Empty.Request()
        self.eat_pizza_cl.call_async(eat_req)


    def spawn_pizza(self, mouse_position):
        req_position = GivePosition.Request()
        req_position.x = mouse_position[0]
        req_position.y = mouse_position[1]
        self.spawn_pizza_cl.call_async(req_position)
        return None

    def mouse_position_calback(self, point : Point):
        self.mouse_position[0] = point.x
        self.mouse_position[1] = point.y
        self.goal_positions.append(np.copy(self.mouse_position))
        self.spawn_pizza(mouse_position=self.mouse_position)

    def pose_callback(self, pose : Pose):
        self.robot_pose[0] = pose.x
        self.robot_pose[1] = pose.y
        self.robot_pose[2] = pose.theta
        q = quaternion_from_euler(0.0, 0.0, self.robot_pose[2])
        # Odometry
        stamp = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = "robot"
        odometry.pose.pose.position.x = self.robot_pose[0]
        odometry.pose.pose.position.y = self.robot_pose[1]
        odometry.pose.pose.orientation.x = q[0]
        odometry.pose.pose.orientation.y = q[1]
        odometry.pose.pose.orientation.z = q[2]
        odometry.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odometry)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = "robot"
        t.transform.translation.x = self.robot_pose[0]
        t.transform.translation.y = self.robot_pose[1]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


    def simple_controller(self, goal_position, index):
        kp_d = 1.5
        kp_w = 3.0
        dp =  goal_position - np.array([self.robot_pose[0], self.robot_pose[1]])
        vx = kp_d * np.linalg.norm(dp)
        e = np.arctan2(dp[1],dp[0]) - self.robot_pose[2]
        w = kp_w * np.arctan2(np.sin(e),np.cos(e))
        if np.linalg.norm(dp) >= 0.5:
            return vx, w
        vx, w = 0.0, 0.0
        self.goal_positions.pop(index)
        return vx, w
        
        
    def timer_callback(self):
        if self.goal_positions:
            vx, w = self.simple_controller(goal_position=self.goal_positions[0], index=0)
            self.cmd_vel(v=vx, w=w)
            self.eat_pizza()
        else:
            self.cmd_vel(v=0.0, w=0.0)

        
    
    def cmd_vel(self, v, w):
        twist : Twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_pub.publish(twist)


    

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()