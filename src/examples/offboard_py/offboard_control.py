#!/usr/bin/env python3

"""
Python implementation of Offboard Control

"""


import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from nav_msgs.msg import Odometry
import nav_msgs

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        
        #some variables
        self.current_pose = None
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode,
                                                                        "/px4_1/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint,
                                                                    "/px4_1/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/px4_1/fmu/in/vehicle_command", 10)
        
        self.subscription = self.create_subscription(Odometry, '/robot/odom_robot', self.odometry_callback, 10)
        
        self.subscription  # prevent unused variable warning

        self.offboard_setpoint_counter_ = 0

        timer_period = 0.1  # 100 milliseconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if (self.offboard_setpoint_counter_ == 10):
            # Change to Offboard mode after 10 setpoints
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            # Arm the vehicle
            self.arm()

        # Offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        #self.odometry_callback(Odometry)

        # stop the counter after reaching 11
        if (self.offboard_setpoint_counter_ < 11):
            self.offboard_setpoint_counter_ += 1
            
            
        '''setpoint_msg = Odometry()
        x = setpoint_msg.pose.pose.position.x
        y = setpoint_msg.pose.pose.position.y
        z = setpoint_msg.pose.pose.position.z'''
        '''out_msg = TrajectorySetpoint()
        out_msg.position = [self.current_pose.pose.pose.position.x, self.current_pose.pose.pose.position.y, self.current_pose.pose.pose.position.z]
        out_msg.yaw = 3.14  # [-PI:PI]
        out_msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(out_msg)'''
        

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

    '''
	Publish a trajectory setpoint
	For this example, it sends a trajectory setpoint to make the
	vehicle hover at 5 meters with a yaw angle of 180 degrees.
    '''

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        #msg.timestamp = self.timestamp_
        msg.position = [0.0, 29.0, -3.0] 
        msg.yaw = 3.14  # [-PI:PI]
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.trajectory_setpoint_publisher_.publish(msg)
        
     
    ''' setting up the UGV odometry topic subscriber callback, and publishing to the trajectory setpoint topic'''
    def odometry_callback(self, msg):
    	self.current_pose = msg
    	'''x = msg.pose.pose.position.x
    	y = msg.pose.pose.position.y
    	z = msg.pose.pose.position.z
    	out_msg = TrajectorySetpoint()
    	out_msg.position = [x, y, z]
    	out_msg.yaw = 3.14  # [-PI:PI]
    	out_msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
    	self.trajectory_setpoint_publisher_.publish(out_msg)'''


    '''
    Publish vehicle commands
        command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
        param1    Command parameter 1 as defined by MAVLink uint16 VEHICLE_CMD enum
        param2    Command parameter 2 as defined by MAVLink uint16 VEHICLE_CMD enum
    '''
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
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
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
