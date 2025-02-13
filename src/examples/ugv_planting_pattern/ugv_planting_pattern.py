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

from std_msgs.msg import Int16
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
        
        #seed level calculation variables
        self.start_time = time.time()
        self.end_time = None
        self.duration = None
        
        #variables to store the UGV orientation
        self.ugv_orientation = None
        self.quaternion = None
        self.ugv_position = None
        
        self.ugvs = 2 #number of UGVs
      
        #Qos settings definitions for publishers and subscribers
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=0)
        
        #creating publishers and subscribers
        #self.vehicle_command_pub_0 = self.create_publisher(Twist, "/robot/cmd_robot", 10)
        
        #self.vehicle_command_pub_1 = self.create_publisher(Twist, "/robot_one/cmd_robot_one", 10)
        
        self.seed_level_publisher = self.create_publisher(Int16, 'int16_topic', 10)
        
        self.subscription1 = self.create_subscription(Odometry, '/robot_0/odom_robot_0', self.odometry_callback, 1)
        
        self.subscription2 = self.create_subscription(Odometry, '/robot_1/odom_robot_1', self.odometry_callback_one, 1)
        

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
        #self.publish_vehicle_command_one()
        
    def seed_level(self):
    	self.start_time = time.time()
    	self.end_time = time.time()
    	self.duration = self.end_time - self.start_time
    
    #function to publish UGV1 command velocity    
    def publish_vehicle_command(self):
    	for i in range(0, self.ugvs):
    		msg = Twist()
    		msg.linear = Vector3()
    		msg.angular = Vector3()
    		
    		seed_level_msg = Int16()
    		
    		ugv_pose_number = "ugv_pose" + str(i)
    		
    		#getting the UGV orientation
    		self.ugv_orientation = getattr(self, ugv_pose_number).pose.pose.orientation
    		self.ugv_position = getattr(self, ugv_pose_number).pose.pose.position
    		
    		self.quaternion = Quaternion()
    		self.quaternion.x = self.ugv_orientation.x
    		self.quaternion.y = self.ugv_orientation.y
    		self.quaternion.z = self.ugv_orientation.z
    		self.quaternion.w = self.ugv_orientation.w
    		
    		#converting UGV orientation from quaternion to euler
    		roll, pitch, yaw = self.euler_from_quaternion(self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w)
    		
    		#condition to turn at the end of the row
    		if self.ugv_position.x >= 5.0:
    			if yaw <= 3.14:
    				msg.linear.x = 0.02
    				msg.linear.y = 0.0
    				msg.linear.z = 0.0
    				
    				msg.angular.x = 0.0
    				msg.angular.y = 0.0
    				msg.angular.z = 2.0
    			
    			
    			else:
    				msg.linear.x = 0.5
    				msg.linear.y = 0.0
    				msg.linear.z = 0.0
    				
    				msg.angular.x = 0.0
    				msg.angular.y = 0.0
    				msg.angular.z = 0.0
    		
    		
    		#condition to turn at the end of the row
    		elif self.ugv_position.x <= -5.0:
    			if yaw >= 0.0:
    				msg.linear.x = 0.02
    				msg.linear.y = 0.0
    				msg.linear.z = 0.0
    				
    				msg.angular.x = 0.0
    				msg.angular.y = 0.0
    				msg.angular.z = -2.0
    			
    			else:
        			msg.linear.x = 0.5
        			msg.linear.y = 0.0
        			msg.linear.z = 0.0
        		
        			msg.angular.x = 0.0
        			msg.angular.y = 0.0
        			msg.angular.z = 0.0
    		
    		
    		#condition to move forward/plant
    		else:
    			msg.linear.x = 0.5
    			msg.linear.y = 0.0
    			msg.linear.z = 0.0
    			
    			msg.angular.x = 0.0
    			msg.angular.y = 0.0
    			msg.angular.z = 0.0
    		
    		#msg.linear = [2.0, 0.0, 0.0]
    		#msg.angular = [0.0, 0.0, 0.0]
    		#msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
    		#ugv_cmd_pub = "vehicle_command_pub_" + str(i)
    		cmd_topic = "robot_"+str(i) + "/cmd_robot_" + str(i) #creating variable to store command topics names
    		
    		self.vehicle_command_pub_ = self.create_publisher(Twist, cmd_topic, 10) #creating the velocity command publisher
    		#getattr(self, ugv_cmd_pub).publish(msg)
    		self.vehicle_command_pub_.publish(msg) #publishing the velocity commands
    		#print("x: ", self.quaternion.x, " y: ", self.quaternion.y, " z: ", self.quaternion.z, " w: ", self.quaternion.w)
    		print("\n yaw: ", yaw)
    		
    		self.end_time = time.time()
    		self.duration = self.end_time - self.start_time
    		print("\n The script has been running for", self.duration, "seconds.")
    		if self.duration >= (10 + (i*20)) and self.duration < (10 + ((i+1)*20)):
    			print("\n UGV", i+1, " needs serving.")
    			seed_level_msg.data = i+1
    			self.seed_level_publisher.publish(seed_level_msg)
    			#I need to add a UGVi_served boolean flag in this condition so that we won't keep saying that UGV1 needs serving everytime duration>10
    		elif self.duration < 10 or self.duration >= (10 + ((self.ugvs)*20)):
    			seed_level_msg.data = 0
    			self.seed_level_publisher.publish(seed_level_msg)
    		
    		if self.duration >= (20 + (self.ugvs*20)):
    			self.start_time = time.time()
    			print("\n The script has been running for", self.duration, "seconds.")
        


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
