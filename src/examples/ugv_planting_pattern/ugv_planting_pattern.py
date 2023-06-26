#!/usr/bin/env python3


"""
Python implementation of Offboard Control
"""


import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
import nav_msgs
import geometry_msgs
from geometry_msgs.msg import Twist, Vector3, Quaternion

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
import math
import numpy as np
import time



class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        
        #variables to receive UGV coordinates
        self.ugvx = 0.0
        self.ugvy = 0.0
        self.ugvz = 0.0
        self.ugv_pose0 = None
        self.ugv_pose1 = None
        
        self.ugv_orientation = None
        self.quaternion = None
      
        
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=0)
        
        
        self.vehicle_command_pub_ = self.create_publisher(Twist, "/robot/cmd_robot", 10)
        
        self.subscription1 = self.create_subscription(Odometry, '/robot/odom_robot', self.odometry_callback, 1)
        
        self.subscription2 = self.create_subscription(Odometry, '/robot_one/odom_robot_one', self.odometry_callback_one, 1)
        

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    ''' setting up the subscribers' callback functions'''
    def odometry_callback(self, msg):
    	self.ugv_pose0 = msg
        
    def odometry_callback_one(self, msg):
        self.ugv_pose1 = msg
        
    
    #function to convert from quaternion to euler
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians    
    
    
    def timer_callback(self):
        # publish vehicle command
        self.publish_vehicle_command()
        
        
    def publish_vehicle_command(self):
        msg = Twist()
        msg.linear = Vector3()
        msg.angular = Vector3()
        
        #getting the UGV orientation
        self.ugv_orientation = self.ugv_pose0.pose.pose.orientation
        self.quaternion = Quaternion()
        self.quaternion.x = self.ugv_orientation.x
        self.quaternion.y = self.ugv_orientation.y
        self.quaternion.z = self.ugv_orientation.z
        self.quaternion.w = self.ugv_orientation.w
        
        roll, pitch, yaw = self.euler_from_quaternion(self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w)
        
        
        if self.ugv_pose0.pose.pose.position.x >= 5.0:
        	if yaw <= 3.14:
        		msg.linear.x = 0.02  # Example value for the x component
        		msg.linear.y = 0.0  # Example value for the y component
        		msg.linear.z = 0.0  # Example value for the z component
        		
        		msg.angular.x = 0.0  # Example value for the x component
        		msg.angular.y = 0.0  # Example value for the y compon
        		msg.angular.z = 2.0  # Example value for the z component
        		
        	else:
        		msg.linear.x = 0.5  # Example value for the x component
        		msg.linear.y = 0.0  # Example value for the y component
        		msg.linear.z = 0.0  # Example value for the z component
        		
        		msg.angular.x = 0.0  # Example value for the x component
        		msg.angular.y = 0.0  # Example value for the y component
        		msg.angular.z = 0.0  # Example value for the z component
        	
        elif self.ugv_pose0.pose.pose.position.x <= -5.0:
        	if yaw >= 0.0:
        		msg.linear.x = 0.02  # Example value for the x component
        		msg.linear.y = 0.0  # Example value for the y component
        		msg.linear.z = 0.0  # Example value for the z component
        		
        		msg.angular.x = 0.0  # Example value for the x component
        		msg.angular.y = 0.0  # Example value for the y compon
        		msg.angular.z = -2.0  # Example value for the z component
        		
        	else:
        		msg.linear.x = 0.5  # Example value for the x component
        		msg.linear.y = 0.0  # Example value for the y component
        		msg.linear.z = 0.0  # Example value for the z component
        		
        		msg.angular.x = 0.0  # Example value for the x component
        		msg.angular.y = 0.0  # Example value for the y component
        		msg.angular.z = 0.0  # Example value for the z component
        	
        else:
        	msg.linear.x = 0.5  # Example value for the x component
        	msg.linear.y = 0.0  # Example value for the y component
        	msg.linear.z = 0.0  # Example value for the z component
        	
        	msg.angular.x = 0.0  # Example value for the x component
        	msg.angular.y = 0.0  # Example value for the y component
        	msg.angular.z = 0.0  # Example value for the z component

        #msg.linear = [2.0, 0.0, 0.0]
        #msg.angular = [0.0, 0.0, 0.0]
        #msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_pub_.publish(msg)
        print("x: ", self.quaternion.x, " y: ", self.quaternion.y, " z: ", self.quaternion.z, " w: ", self.quaternion.w)
        print("\n yaw: ", yaw)



def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    
    '''offboard_control.iterate()
    print("Cost adjacency matrix: \n")
    print(offboard_control.dist_matrix)'''
    
    rclpy.spin(offboard_control)
    
    '''offboard_control.iterate()
    print("Cost adjacency matrix: \n")
    print(offboard_control.dist_matrix)'''

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
