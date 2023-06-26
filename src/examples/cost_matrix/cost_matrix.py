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
        self.dist_matrix = np.zeros((2, 2), float)
        self.ugv_to_be_served = 0
        self.uav_to_serve = 0
        self.close_pair = 1000
        
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
        
        self.k = 1
        self.var_name = "/px4_{}/fmu/out/vehicle_local_position".format(self.k)
        self.offboard_mode_topics = None
        self.trajectory_set_point_topics = None
        self.vehicle_command_topics = None
        
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=0)
        
        '''self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/px4_1/fmu/in/offboard_control_mode", 1)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/px4_1/fmu/in/trajectory_setpoint", 1)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 1)'''
        
        self.subscription1 = self.create_subscription(Odometry, '/robot/odom_robot', self.odometry_callback, 1)
        
        self.subscription2 = self.create_subscription(Odometry, '/robot_one/odom_robot_one', self.odometry_callback_one, 1)
        
        self.subscription3 = self.create_subscription(VehicleLocalPosition, self.var_name, self.TrajectorySetpoint_callback1, qos_profile_sub)
        
        self.subscription4 = self.create_subscription(VehicleLocalPosition, "/px4_2/fmu/out/vehicle_local_position", self.TrajectorySetpoint_callback2, qos_profile_sub)

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
    
    
    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.iterate()
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        #print("xa:", self.xa, " ya:", self.ya, " za:", self.za)
        #print("xg:", self.xg, " yg:", self.yg, " zg:", self.zg)
        
        varg_name = "ugv_pose" + str(self.ugv_to_be_served - 1)
        vara_name = "uav_pose" + str(self.uav_to_serve - 1)
        xaa = getattr(self, vara_name).x
        zaa = getattr(self, vara_name).z
        xgg = getattr(self, varg_name).pose.pose.position.y
        yaa = getattr(self, vara_name).y
        ygg = getattr(self, varg_name).pose.pose.position.x
        
        print("xaa:", xaa, " zaa:", zaa, " xgg:", xgg, " yaa:", yaa, " ygg:", ygg)
        
        if abs(ygg - yaa)<=0.25 and round(zaa, 0) == -2.0:
        	#self.publish_trajectory_setpoint()
        	#self.publish_trajectory_setpoint()
        	#self.refill()
        	#time.sleep(2)
        	self.return_to_base()
        	time.sleep(8)
        	self.disarm()

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

    	
    
    
    '''def iterate(self):
        if self.uav_pose0 is not None:
            xa = self.uav_pose0.x
            print(xa)
        else:
            print("Waiting for uav_pose0 to be updated...")'''
            
    #The function to derive and store the cost adjacency matrix
    def iterate(self):
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

                    	#calculating the distance
                    	dist = math.sqrt((self.xg-self.xa)**2 + (self.yg-self.ya)**2 + (0.0-0.0)**2)
                    	self.dist_matrix[j][i] = dist
            	
            	
            	else:
            		print("waiting for position info \n")
                    	#print("xa:", xa, " ya:", ya, " za:", za)
          
                	
        print("\n Cost adjacency matrix:")
        print(self.dist_matrix)
        
        #loops below are for choosing the pair to be serviced based on how close their. this is just used to test how the final code will look like
        for a in range(0, 2):
            for b in range(0, 2):
            	if self.dist_matrix[a][b] < self.close_pair:
            		#finding the closest UAV-UGV pair and storing the pair's info
                    	self.close_pair = self.dist_matrix[a][b]
                    	self.uav_to_serve = a + 1
                    	self.ugv_to_be_served = b + 1

                    	self.offboard_mode_topics = "/px4_{}/fmu/in/offboard_control_mode".format(self.uav_to_serve)
                    	self.trajectory_set_point_topics = "/px4_{}/fmu/in/trajectory_setpoint".format(self.uav_to_serve)
                    	self.vehicle_command_topics = "/px4_{}/fmu/in/vehicle_command".format(self.uav_to_serve)
        
        print("UAV", self.uav_to_serve," going to serve UGV", self.ugv_to_be_served)
        
        #creating publishers for the chosen UAV
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        self.offboard_mode_topics, 1)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    self.trajectory_set_point_topics, 1)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, self.vehicle_command_topics, 1)    		
      
        
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
        
        
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        #msg.timestamp = self.timestamp_
        x_offset = -1*(3.15+(self.uav_to_serve - 1)*2.35)
        
        if self.ugv_to_be_served == 1:
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
        	
    def return_to_base(self):
        msg = TrajectorySetpoint()
        #msg.timestamp = self.timestamp_
        x_offset = -1*(3.15+(self.uav_to_serve - 1)*2.35)
        
        if self.ugv_to_be_served == 1:
        	msg.position = [-0.002299, -0.0419322, -1.0]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)

        elif self.ugv_to_be_served == 2:
        	msg.position = [-0.002299, -0.0419322, -1.0]
        	msg.yaw = -3.14  # [-PI:PI]
        	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        	self.trajectory_setpoint_publisher_.publish(msg)
        	
        	
    '''def refill(self):
    	msg = TrajectorySetpoint()
    	#msg.timestamp = self.timestamp_
    	x_offset = -1*(3.15+(self.uav_to_serve - 1)*2.35)
    	msg.position = [self.ugvx, self.ugvy, -2.0]
    	msg.yaw = -3.14  # [-PI:PI]
    	msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
    	self.trajectory_setpoint_publisher_.publish(msg)
    	time.sleep(2)'''
    
        
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = self.uav_to_serve + 1  # system which should execute the command
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
