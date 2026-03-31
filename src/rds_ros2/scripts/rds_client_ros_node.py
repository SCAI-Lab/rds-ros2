#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rds_msgs.srv import VelocityCommandCorrectionRDS
import math

class RDSVelocityModulator(Node):
    def __init__(self):
        super().__init__('rds_velocity_modulator')
        
        # Publishers and Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_in', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'rds_modulated_cmd_vel', 10)
        
        # Service client for RDS
        self.rds_client = self.create_client(
            VelocityCommandCorrectionRDS, 'rds_velocity_command_correction')
        
        # State variables
        self.latest_cmd_vel = Twist()
        self.latest_odom = None
        self.last_time = self.get_clock().now()
        
        # RDS Parameters
        self.capsule_center_front_y = 0.18
        self.capsule_center_rear_y = -0.5
        self.capsule_radius = 0.2
        self.reference_point_y = 0.18
        self.rds_tau = 1.5
        self.rds_delta = 0.1
        self.vel_lim_linear_min = -0.5
        self.vel_lim_linear_max = 2.0
        self.vel_lim_angular_abs_max = 1.0
        self.vel_linear_at_angular_abs_max = 0.2
        self.acc_limit_linear_abs_max = 3.0
        self.acc_limit_angular_abs_max = 3.0
        self.dt = 0.1
        self.lrf_point_obstacles = True
        self.ORCA_implementation = False
        
        # Timer for regular RDS calls
        self.timer = self.create_timer(0.1, self.rds_correction_timer)  # 10 Hz
        
        self.get_logger().info('RDS Velocity Modulator Node Started')
        
        # Wait for RDS service
        while not self.rds_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for RDS service...')

    def cmd_vel_callback(self, msg):
        """Store the latest command velocity from teleop."""
        self.latest_cmd_vel = msg

    def odom_callback(self, msg):
        """Store the latest odometry data."""
        self.latest_odom = msg

    def rds_correction_timer(self):
        """Regular timer to call RDS service and publish corrected velocity."""
        if not self.rds_client.service_is_ready():
            self.get_logger().warn('RDS service not ready')
            return
            
        # Create service request
        request = VelocityCommandCorrectionRDS.Request()
        
        # Fill in the nominal command from teleop
        request.nominal_command.linear = float(self.latest_cmd_vel.linear.x)
        request.nominal_command.angular = float(self.latest_cmd_vel.angular.z)
        
        # Fill in RDS parameters (matching the C++ code)
        request.capsule_center_front_y = self.capsule_center_front_y
        request.capsule_center_rear_y = self.capsule_center_rear_y
        request.capsule_radius = self.capsule_radius
        request.reference_point_y = self.reference_point_y
        request.rds_tau = self.rds_tau
        request.rds_delta = self.rds_delta
        request.vel_lim_linear_min = self.vel_lim_linear_min
        request.vel_lim_linear_max = self.vel_lim_linear_max
        request.vel_lim_angular_abs_max = self.vel_lim_angular_abs_max
        request.vel_linear_at_angular_abs_max = self.vel_linear_at_angular_abs_max
        request.acc_limit_linear_abs_max = self.acc_limit_linear_abs_max
        request.acc_limit_angular_abs_max = self.acc_limit_angular_abs_max
        request.dt = self.dt
        request.lrf_point_obstacles = self.lrf_point_obstacles
        request.orca_implementation = self.ORCA_implementation  
        
        # Call the service asynchronously
        future = self.rds_client.call_async(request)
        future.add_done_callback(self.rds_response_callback)

    def rds_response_callback(self, future):
        """Handle the RDS service response."""
        try:
            response = future.result()
            
            # Create corrected velocity command
            corrected_cmd = Twist()
            corrected_cmd.linear.x = response.corrected_command.linear
            corrected_cmd.angular.z = response.corrected_command.angular
            
            # Publish the corrected command
            self.cmd_vel_pub.publish(corrected_cmd)
            
            # Log the correction (optional)
            if abs(response.corrected_command.linear - self.latest_cmd_vel.linear.x) > 0.01 or \
               abs(response.corrected_command.angular - self.latest_cmd_vel.angular.z) > 0.01:
                self.get_logger().info(
                    f'RDS Correction: '
                    f'Linear: {self.latest_cmd_vel.linear.x:.3f} -> {response.corrected_command.linear:.3f}, '
                    f'Angular: {self.latest_cmd_vel.angular.z:.3f} -> {response.corrected_command.angular:.3f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'RDS service call failed: {e}')
            # Publish original command as fallback
            self.cmd_vel_pub.publish(self.latest_cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = RDSVelocityModulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()