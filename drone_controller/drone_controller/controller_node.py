#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.start_time = self.get_clock().now().nanoseconds

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        self.current_z = 0.0
        self.current_y= 0.0
        self.current_x=0.0

        
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        
        self.arm_timer = self.create_timer(1.0, self.arm)
        self.mode_timer = self.create_timer(1.0, self.set_offboard_mode)
        self.offboard_timer = self.create_timer(0.1, self.publish_offboard_mode)
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)

    def odom_callback(self, msg):
        self.current_z = msg.position[2]
        self.current_y= msg.position[1]
        self.current_x=msg.position[0]
        self.get_logger().info(f"Z position: {self.current_z}, Y position: {self.current_y}, X position: {self.current_x}")

    def publish_offboard_mode(self):
        msg = OffboardControlMode()

        msg.position = True   
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.offboard_mode_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()

        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_pub.publish(msg)

    def set_offboard_mode(self):
        msg = VehicleCommand()

        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0  

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_pub.publish(msg)

    def publish_setpoint(self):
        msg = TrajectorySetpoint()

        t= (self.get_clock().now().nanoseconds - self.start_time ) / 1e9

        if t<10:
            x=0.0
            y=0.0
            z=-1.0
        elif t<20:
            x=1.0
            y=0.0
            z=-1.0
        else:
            x=1.0
            y=1.0
            z=-1.0 

        msg.position= [x,y,z]
        msg.yaw=0.0
        msg.timestamp= int(self.get_clock().now().nanoseconds / 1000)

        self.setpoint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()