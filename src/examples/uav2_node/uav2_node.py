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

import math
import numpy as np
import time



class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        
        #some variables
        self.ugv_pose0 = None
        self.ugv_pose1 = None
        self.ugv_pose2 = None
        self.uav_pose0 = None
        self.uav_status0 = None #variable for storing UAV2 arming status
        self.ugv_to_be_served_msg = None #variable to receive the info of the UGV that needs to be served
        self.ugv_to_be_served = None #previously initialized to zero. change back if you get relevant errors
        
        
        #variables to receive UGV coordinates
        self.ugvx = 0.0
        self.ugvy = 0.0
        self.ugvz = 0.0
        
        
        #Qos settings definitions for publishers and subscribers
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=0)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/px4_2/fmu/in/offboard_control_mode", 1)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/px4_2/fmu/in/trajectory_setpoint", 1)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_2/fmu/in/vehicle_command", 1)
        
        #subscribers to the UGVs and UAVs position and status topics
        self.subscription1 = self.create_subscription(Odometry, '/robot_0/odom_robot_0', self.odometry_callback, 1)
        
        self.subscription2 = self.create_subscription(Odometry, '/robot_1/odom_robot_1', self.odometry_callback_one, 1)
        
        self.subscription3 = self.create_subscription(VehicleLocalPosition, "/px4_2/fmu/out/vehicle_local_position", self.TrajectorySetpoint_callback1, qos_profile_sub)
        
        self.subscription5 = self.create_subscription(VehicleStatus, "/px4_2/fmu/out/vehicle_status", self.VehicleStatus_callback1, qos_profile_sub)
        
        self.subscription7 = self.create_subscription(Int16, '/serving_uav_topic2', self.ugv_to_be_served_callback, 10) #subscriber to the topic that publishes the UGV that needs to be served

        self.offboard_setpoint_counter_ = 0

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    ''' setting up the subscribers' callback functions'''
    def odometry_callback(self, msg):
    	self.ugv_pose0 = msg
        
    def odometry_callback_one(self, msg):
        self.ugv_pose1 = msg
        
    def TrajectorySetpoint_callback1(self, msg):
        self.uav_pose0 = msg
    
    def VehicleStatus_callback1(self, msg):
        self.uav_status0 = msg
    
    def ugv_to_be_served_callback(self, msg):
        self.ugv_to_be_served_msg = msg
    
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
        	print("\n UAV2 going to serve UGV", self.ugv_to_be_served)
        	
        	if self.ugv_to_be_served == 1 or self.ugv_to_be_served == 2:
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
        		ygg = getattr(self, varg_name).pose.pose.position.x
        		
        		print("xaa:", xaa, " zaa:", zaa, " xgg:", xgg, " yaa:", yaa, " ygg:", ygg)
        		
        		#condition for the UAV to fly back to the ground station after refilling
        		
        		if abs(ygg - yaa)<=0.35 and round(zaa, 0) == -2.0:
        			#self.publish_trajectory_setpoint()
        			#self.publish_trajectory_setpoint()
        			#self.refill()
        			#time.sleep(2)
        			self.return_to_base()
        			time.sleep(8)
        			self.disarm()
        			time.sleep(2)
        
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
        x_offset = -1*(5.5)
        
        print("\n UAV2 going to serve UGV", self.ugv_to_be_served)
        
        if self.ugv_to_be_served == 0:
        	print("\n No UGV needs serving")
        
        elif self.ugv_to_be_served == 1:
        	self.ugvx = self.ugv_pose0.pose.pose.position.y + x_offset
        	self.ugvy = self.ugv_pose0.pose.pose.position.x - 0.15
        	msg.position = [self.ugvx, self.ugvy, -2.0]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)

        elif self.ugv_to_be_served == 2:
        	self.ugvx = self.ugv_pose1.pose.pose.position.y + x_offset
        	self.ugvy = self.ugv_pose1.pose.pose.position.x - 0.15
        	msg.position = [self.ugvx, self.ugvy, -2.0]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)
    
    #function for the UAV to fly back to the ground station after serving    	
    def return_to_base(self):
        msg = TrajectorySetpoint()
        #msg.timestamp = self.timestamp_
        self.ugv_to_be_served = int(self.ugv_to_be_served_msg.data)
        x_offset = -1*(5.5)
        
        if self.ugv_to_be_served == 0:
        	print("\n No UGV needs serving")
        
        elif self.ugv_to_be_served == 1:
        	self.ugvy = self.ugv_pose0.pose.pose.position.x - 0.15
        	msg.position = [-13.5, 12.5, -1.5]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)

        elif self.ugv_to_be_served == 2:
        	self.ugvy = self.ugv_pose1.pose.pose.position.x - 0.15
        	msg.position = [-13.5, 12.5, -1.5]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)
        	
    
    #function to publish velocity command to the UAV    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
    	if self.ugv_to_be_served == 0:
    		print("\n No UGV needs serving because our data is: ", self.ugv_seed_level.data)
    	
    	else:
    		msg = VehicleCommand()
    		msg.param1 = param1
    		msg.param2 = param2
    		msg.command = command  # command ID
    		msg.target_system = 3  # system which should execute the command
    		msg.target_component = 1  # component which should execute the command, 0 for all components
    		msg.source_system = 1  # system sending the command
    		msg.source_component = 1  # component sending the command
    		msg.from_external = True
    		msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
    		self.vehicle_command_publisher_.publish(msg)





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
