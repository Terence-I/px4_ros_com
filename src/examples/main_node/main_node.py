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
        self.uav_pose1 = None
        self.uav_status0 = None #variable for storing UAV1 arming status
        self.uav_status1 = None #variable for storing UAV2 arming status
        self.ugv_seed_level = 0 #variable to receive the info of the UGV that needs to be served
        self.ugvs = 2 #variable to store the number of UGVs
        self.uavs = 2 #variable to store the number of UAVs
        
        self.dist_matrix = np.zeros((2, 2), float)
        self.ugv_to_be_served = None #previously initialized to zero. change back if you get relevant errors
        self.uav_to_serve = None
        self.close_pair = 1000
        self.iterate_flag = 0 #variable to check how many times we have run the iterate method (might not be necessary)
        
        #ugv and uav position global variables (you must end up not using them, if you don't delete them)
        self.xa =0.0
        self.ya =0.0
        self.za =0.0
        self.xg =0.0
        self.yg =0.0
        self.zg =0.0
        
        #variables to receive UGV coordinates
        self.ugvx = 0.0
        self.ugvy = 0.0
        self.ugvz = 0.0
        
        #seed level calculation variables
        self.start_time = time.time()
        self.end_time = None
        self.duration = None
        
        #variables to store names of the UAV topics
        self.k = 1
        self.var_name = "/px4_{}/fmu/out/vehicle_local_position".format(self.k)
        self.offboard_mode_topics = None
        self.trajectory_set_point_topics = None
        self.vehicle_command_topics = None
        
        #variable to store whether a ugv is being served
        self.ugv_being_served = np.zeros((1,2), bool)
        
        
        #Qos settings definitions for publishers and subscribers
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=0)
        
        '''self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/px4_1/fmu/in/offboard_control_mode", 1)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/px4_1/fmu/in/trajectory_setpoint", 1)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 1)'''
        
        #subscribers to the UGVs and UAVs position and status topics
        self.subscription1 = self.create_subscription(Odometry, '/robot_0/odom_robot_0', self.odometry_callback, 1)
        
        self.subscription2 = self.create_subscription(Odometry, '/robot_1/odom_robot_1', self.odometry_callback_one, 1)
        
        self.subscription3 = self.create_subscription(VehicleLocalPosition, self.var_name, self.TrajectorySetpoint_callback1, qos_profile_sub)
        
        self.subscription4 = self.create_subscription(VehicleLocalPosition, "/px4_2/fmu/out/vehicle_local_position", self.TrajectorySetpoint_callback2, qos_profile_sub)
        
        self.subscription5 = self.create_subscription(VehicleStatus, "/px4_1/fmu/out/vehicle_status", self.VehicleStatus_callback1, qos_profile_sub)
        
        self.subscription6 = self.create_subscription(VehicleStatus, "/px4_2/fmu/out/vehicle_status", self.VehicleStatus_callback2, qos_profile_sub)
        
        self.subscription7 = self.create_subscription(Int16, '/int16_topic', self.seed_level_callback, 10) #subscriber for to the topic that publishes the UGV that needs to be served
        
        #publishers to publish ugv_to_be_served to the respective chosen uav_to_serve node
        self.serving_uav_publisher1 = self.create_publisher(Int16, 'serving_uav_topic1', 10)
        
        self.serving_uav_publisher2 = self.create_publisher(Int16, 'serving_uav_topic2', 10)

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
        
    def TrajectorySetpoint_callback2(self, msg):
        self.uav_pose1 = msg
    
    def VehicleStatus_callback1(self, msg):
        self.uav_status0 = msg
    
    def VehicleStatus_callback2(self, msg):
        self.uav_status1 = msg
    
    def seed_level_callback(self, msg):
        self.ugv_seed_level = msg
    
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
    	self.publish_ugv_vehicle_command()
    	
    	#going through the digraph model to find the serving pair
    	self.iterate()
    
    
    #function to publish UGVs command velocity    
    def publish_ugv_vehicle_command(self):
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
    				msg.linear.x = 0.35
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
        			msg.linear.x = 0.35
        			msg.linear.y = 0.0
        			msg.linear.z = 0.0
        		
        			msg.angular.x = 0.0
        			msg.angular.y = 0.0
        			msg.angular.z = 0.0
    		
    		
    		#condition to move forward/plant
    		else:
    			msg.linear.x = 0.35
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
    			self.ugv_to_be_served = i+1
    		elif self.duration < 10 or self.duration >= (10 + ((self.ugvs)*20)):
    			self.ugv_to_be_served = 0
    		
    		if self.duration >= (20 + (self.ugvs*20)):
    			self.start_time = time.time()
    			print("\n The script has been running for", self.duration, "seconds.")
    			self.ugv_being_served[:] = False #initializing the ugv being served status to false for all UGVs
    			print("\n initializing ugv being served status to false again (in seed_level)", self.ugv_being_served)
    
    
    
    
    
            
    
    #The function to derive and store the cost adjacency matrix
    def iterate(self):
        
        self.iterate_flag = self.iterate_flag + 1
        for j in range(0, 2):
            for i in range(0, 2):
            	if j == 0 and self.uav_pose0 is not None:
            		#getting UAV position info
                    	self.xa = self.uav_pose0.x + (-3.15)
                    	self.ya = self.uav_pose0.y + 0.34
                    	self.za = self.uav_pose0.z

                    	print("xa:", self.xa, " ya:", self.ya, " za:", self.za)

                    	#getting UGV position info (changing from ENU to NED coordinate system as well)
                    	var_name = "ugv_pose" + str(i)
                    	self.xg = getattr(self, var_name).pose.pose.position.y
                    	self.yg = getattr(self, var_name).pose.pose.position.x
                    	self.zg = -1 * getattr(self, var_name).pose.pose.position.z

                    	print("xg:", self.xg, " yg:", self.yg, " zg:", self.zg)

                    	#calculating the distance
                    	dist = math.sqrt((self.xg-self.xa)**2 + (self.yg-self.ya)**2 + (0.0-0.0)**2)
                    	self.dist_matrix[j][i] = dist
                    	#var_value = getattr(self, var_name)
                    	#self.dist_list.append(var_value)
                    	#self.dist_matrix[j][i] = getattr(self, var_name)
            	
            	
            	elif j == 1 and self.uav_pose1 is not None:
            		#getting UAV position info
                    	self.xa = self.uav_pose1.x + (-5.5)
                    	self.ya = self.uav_pose1.y + 0.3
                    	self.za = self.uav_pose1.z

                    	print("xa1:", self.xa, " ya1:", self.ya, " za1:", self.za)

                    	#getting UGV position info (changing from ENU to NED coordinate system as well)
                    	var_name = "ugv_pose" + str(i)
                    	self.xg = getattr(self, var_name).pose.pose.position.y
                    	self.yg = getattr(self, var_name).pose.pose.position.x
                    	self.zg = -1 * getattr(self, var_name).pose.pose.position.z

                    	print("xg1:", self.xg, " yg1:", self.yg, " zg1:", self.zg)

                    	#calculating the distance and storing in the matrix
                    	dist = math.sqrt((self.xg-self.xa)**2 + (self.yg-self.ya)**2 + (0.0-0.0)**2)
                    	self.dist_matrix[j][i] = dist
            	
            	
            	else:
            		print("waiting for position info \n")
                    	#print("xa:", xa, " ya:", ya, " za:", za)
          
                	
        print("\n Cost adjacency matrix:")
        print(self.dist_matrix)
        
        #loops below are for choosing the pair to be serviced based on how close they are. this is just used to test how the final code will look like, replace with digraph model
        
        
        for a in range(0, 2):
        	if self.ugv_to_be_served == 0:
        		print("\n No UGV needs serving because our data is: 0")
        	
        	else:
        		shortest_distance = np.min(self.dist_matrix[:, self.ugv_to_be_served - 1])
        		min_indices = np.argwhere(self.dist_matrix == shortest_distance)
        		min_row, min_col = min_indices[0]  # Extract row and column indices
        		#first create this variable chosen_uav = int(min_row + 1)
        		chosen_uav = int(min_row + 1)
        		#then put a condition here: if chosen_uav is not serving (use getattr to access the right topic) then assign to uav_to_serve and publish the ugv_to_be_served to the respective UAV topic
        		is_it_armed = "uav_status" + str(chosen_uav - 1)
        		self.serving_status = getattr(self, is_it_armed).arming_state
        		
        		#condition to check whether the UAV and UGV in the chosen pair is serving of being served respectively
        		if self.serving_status == 1 and self.ugv_being_served[0][self.ugv_to_be_served - 1] == False:
        			self.uav_to_serve = chosen_uav
        			#create topic to publish ugv_to_be_served to the chosen UAV
        			serving_uav_pub = "serving_uav_publisher" + str(self.uav_to_serve)
        			serve_this_ugv_msg = Int16()
        			serve_this_ugv_msg.data = self.ugv_to_be_served
        			getattr(self, serving_uav_pub).publish(serve_this_ugv_msg) #publishing the UGV to be served to the appropriate UAV topic
        			print("UAV", self.uav_to_serve," going to serve UGV", self.ugv_to_be_served)
        			print("\n ugv being served status", self.ugv_being_served)
        			self.ugv_being_served[0][self.ugv_to_be_served - 1] = True #changing the UGV being served status of the ugv_to_be_served to True
        			print("\n the updated ugv being served status", self.ugv_being_served)
        			break
        		
        		
        		elif self.serving_status == 1 and self.ugv_being_served[0][self.ugv_to_be_served - 1] == True:
        			print("\n uav", chosen_uav, " not going to serve bcz the updated ugv being served status is", self.ugv_being_served)
        		
        		#else if chosen_uav is serving change the corresponding matrix cell to a very high value and iterate again to find the next best uav_to_serve
        		elif self.serving_status == 2:
        			self.dist_matrix[chosen_uav - 1][self.ugv_to_be_served - 1] = 1000
        			print("finding another UAV because UAV", chosen_uav, "is currently serving \n")        
    
 



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
