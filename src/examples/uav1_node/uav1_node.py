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

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

from std_msgs.msg import Int16
from std_msgs.msg import Bool  # Import the standard boolean message type

import math
import numpy as np
import time



class Uav1Node(Node):

    def __init__(self):
        super().__init__('Uav1Node')
        
        #some variables
        self.ugv_pose0 = None
        self.ugv_pose1 = None
        self.ugv_pose2 = None
        self.ugv_pose3 = None
        self.ugv_pose4 = None
        
        
        self.uav_pose0 = None
        self.station_pose = None #variable to store position information of the uav ground station
        self.uav_status0 = None #variable for storing UAV1 arming status
        self.ugv_to_be_served_msg = None #variable to receive the info of the UGV that needs to be served
        self.ugv_to_be_served = 0 #previously initialized to None. change back if you get relevant errors
        
        self.can_serve_flag = True #flag to check whether a UAV received a new service request
        
        
        #variables to receive UGV coordinates
        self.ugvx = 0.0
        self.ugvy = 0.0
        self.ugvz = 0.0
        
        self.ygg = 0.0
        
        
        #Qos settings definitions for publishers and subscribers
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=0)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/px4_1/fmu/in/offboard_control_mode", 1)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/px4_1/fmu/in/trajectory_setpoint", 1)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 1)
        
        #subscribers to the UGVs and UAVs position and status topics
        self.subscription1 = self.create_subscription(Odometry, '/robot_0/odom_robot_0', self.odometry_callback, 1)
        
        self.subscription2 = self.create_subscription(Odometry, '/robot_1/odom_robot_1', self.odometry_callback_one, 1)
        
        self.subscription4 = self.create_subscription(Odometry, '/robot_2/odom_robot_2', self.odometry_callback_two, 1)
        
        self.subscription8 = self.create_subscription(Odometry, '/robot_3/odom_robot_3', self.odometry_callback_three, 1)
        
        self.subscription9 = self.create_subscription(Odometry, '/robot_4/odom_robot_4', self.odometry_callback_four, 1)
        
        self.subscription6 = self.create_subscription(Odometry, '/uav_station/odom_uav_station', self.odometry_station_callback, 1)
        
        self.subscription3 = self.create_subscription(VehicleLocalPosition, "/px4_1/fmu/out/vehicle_local_position", self.TrajectorySetpoint_callback1, qos_profile_sub)
        
        self.subscription5 = self.create_subscription(VehicleStatus, "/px4_1/fmu/out/vehicle_status", self.VehicleStatus_callback1, qos_profile_sub)
        
        self.subscription7 = self.create_subscription(Int16, '/serving_uav_topic1', self.ugv_to_be_served_callback, 0) #subscriber to the topic that publishes the UGV that needs to be served

        self.offboard_setpoint_counter_ = 0

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    ''' setting up the subscribers' callback functions'''
    def odometry_callback(self, msg):
    	self.ugv_pose0 = msg
        
    def odometry_callback_one(self, msg):
        self.ugv_pose1 = msg
        
    def odometry_callback_two(self, msg):
        self.ugv_pose2 = msg
    
    def odometry_callback_three(self, msg):
        self.ugv_pose3 = msg
    
    def odometry_callback_four(self, msg):
        self.ugv_pose4 = msg
    
    def TrajectorySetpoint_callback1(self, msg):
        self.uav_pose0 = msg
    
    def VehicleStatus_callback1(self, msg):
        self.uav_status0 = msg
    
    def ugv_to_be_served_callback(self, msg):
    	self.ugv_to_be_served_msg = msg
    	self.can_serve_flag = True #changing the flag to check whether a UAV received a new service request to true
    
    def odometry_station_callback(self, msg):
        self.station_pose = msg
    
    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()
        
        #self.ugv_to_be_served = self.ugv_to_be_served_msg.data
        
        print("\n waiting for ugv_to_be_served message")
        
        if self.ugv_to_be_served_msg is not None:
        	self.ugv_to_be_served = int(self.ugv_to_be_served_msg.data)
        	
        	if self.ugv_to_be_served != 0 and self.can_serve_flag == True:
        		print("\n UAV1 going to serve UGV", self.ugv_to_be_served)
        		# Offboard_control_mode needs to be paired with trajectory_setpoint
        		self.arm()
        		self.publish_offboard_control_mode()
        		self.publish_trajectory_setpoint()
        		
        		#saving the UGV to be served and UAV to serve position info in variables. I added -1 because I initialized the uav to serve and ugv to beserved variables to 1 to eliminate an error
        		varg_name = "ugv_pose" + str(self.ugv_to_be_served - 1)
        		xaa = self.uav_pose0.x
        		zaa = self.uav_pose0.z
        		xgg = getattr(self, varg_name).pose.pose.position.y
        		yaa = self.uav_pose0.y
        		self.ygg = getattr(self, varg_name).pose.pose.position.x
        		
        		print("xaa:", xaa, " zaa:", zaa, " xgg:", xgg, " yaa:", yaa, " ygg:", self.ygg)
        		
        		#condition for the UAV to fly back to the ground station after refilling
        		
        		if abs(self.ygg - yaa)<=0.35 and round(zaa, 0) == -3.0:
        			#self.publish_trajectory_setpoint()
        			#self.publish_trajectory_setpoint()
        			#self.refill()
        			#time.sleep(2)
        			self.return_to_base()
        			time.sleep(10)
        			self.disarm()
        			time.sleep(2)
        			self.can_serve_flag = False #changing the flag to check whether a UAV received a new service request to false
        
        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1

    # Arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")

    	
    
        
    '''
	Publish the offboard control mode.
	For this example, only position and altitude controls are active.
    '''
    
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.offboard_control_mode_publisher_.publish(msg)
        
    # function for flying the chosen UAV to serve, to the UGV to be served    
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        #msg.timestamp = self.timestamp_
        self.ugv_to_be_served = int(self.ugv_to_be_served_msg.data)
        x_offset = -1*(3.15)
        
        print("\n UAV1 going to serve UGV", self.ugv_to_be_served)
        
        if self.ugv_to_be_served == 0:
        	print("\n No UGV needs serving")
        
        else:
        	ugv_name = "ugv_pose" + str(self.ugv_to_be_served - 1)
        	self.ugvx = getattr(self, ugv_name).pose.pose.position.y + x_offset
        	self.ugvy = getattr(self, ugv_name).pose.pose.position.x - 0.15
        	msg.position = [self.ugvx, self.ugvy, -3.0]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)
    
    #function for the UAV to fly back to the ground station after serving    	
    def return_to_base(self):
        msg = TrajectorySetpoint()
        #msg.timestamp = self.timestamp_
        self.ugv_to_be_served = int(self.ugv_to_be_served_msg.data)
        x_offset = -1*(4.15)
        
        if self.ugv_to_be_served == 0:
        	print("\n No UGV needs serving")
        
        else:
        	msg.position = [(self.station_pose.pose.pose.position.y) - x_offset, self.station_pose.pose.pose.position.x + 2.0, -1.5]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)
        	
        	if abs(self.station_pose.pose.pose.position.y - self.uav_pose0.x)<=0.1 and abs(self.station_pose.pose.pose.position.x - self.uav_pose0.y)<=0.3:
        		self.disarm()
        		#time.sleep(3)
        	
    
    #function to publish velocity command to the UAV    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
    	if self.ugv_to_be_served == 0:
    		print("\n No UGV needs serving because our data is: ", self.ugv_to_be_served)
    	
    	else:
    		msg = VehicleCommand()
    		msg.param1 = param1
    		msg.param2 = param2
    		msg.command = command  # command ID
    		msg.target_system = 2  # system which should execute the command
    		msg.target_component = 1  # component which should execute the command, 0 for all components
    		msg.source_system = 1  # system sending the command
    		msg.source_component = 1  # component sending the command
    		msg.from_external = True
    		msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
    		self.vehicle_command_publisher_.publish(msg)





def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = Uav1Node()
    
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
