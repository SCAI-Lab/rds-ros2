#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Arrow, Circle
import math
import threading
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class RDSVisualization(Node):
    def __init__(self):
        super().__init__('rds_visualization')
        
        # Declare parameters
        self.declare_parameter('scan_topics', ['/scan_left', '/scan_right'])
        self.declare_parameter('nominal_cmd_topic', '/remote_cmd_vel')
        self.declare_parameter('corrected_cmd_topic', '/rds_modulated_cmd_vel')
        self.declare_parameter('max_range', 10.0)
        
        # Get parameters
        self.scan_topics = self.get_parameter('scan_topics').value
        self.nominal_cmd_topic = self.get_parameter('nominal_cmd_topic').value
        self.corrected_cmd_topic = self.get_parameter('corrected_cmd_topic').value
        self.max_range = self.get_parameter('max_range').value
        
        # Initialize data storage
        self.scan_data = {}
        self.nominal_cmd = Twist()
        self.corrected_cmd = Twist()
        
        # Create subscriptions
        for topic in self.scan_topics:
            self.create_subscription(
                LaserScan, 
                topic, 
                lambda msg, topic=topic: self.scan_callback(msg, topic),
                10)
        
        self.create_subscription(
            Twist,
            self.nominal_cmd_topic,
            self.nominal_cmd_callback,
            10)
        
        self.create_subscription(
            Twist,
            self.corrected_cmd_topic,
            self.corrected_cmd_callback,
            10)
        
        # Setup visualization
        self.setup_visualization()
        
        self.get_logger().info("RDS Visualization started")
        self.get_logger().info(f"Monitoring scan topics: {self.scan_topics}")
        self.get_logger().info(f"Monitoring commands: {self.nominal_cmd_topic} -> {self.corrected_cmd_topic}")
    
    def scan_callback(self, msg, topic):
        # Convert scan to cartesian coordinates
        points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
        
        # Store scan data
        self.scan_data[topic] = points
    
    def nominal_cmd_callback(self, msg):
        self.nominal_cmd = msg
    
    def corrected_cmd_callback(self, msg):
        self.corrected_cmd = msg
    
    def setup_visualization(self):
        # Create tkinter window
        self.root = tk.Tk()
        self.root.title("RDS Environment and Command Visualization")
        self.root.geometry("1000x800")
        
        # Create matplotlib figure
        self.fig = plt.figure(figsize=(10, 8))
        
        # Create 2D map subplot
        self.ax1 = self.fig.add_subplot(1, 1, 1)
        self.ax1.set_title("RDS Environment and Commands")
        self.ax1.set_xlabel("X (meters)")
        self.ax1.set_ylabel("Y (meters)")
        self.ax1.grid(True)
        self.ax1.set_xlim(-self.max_range, self.max_range)
        self.ax1.set_ylim(-self.max_range, self.max_range)
        self.ax1.set_aspect('equal')
        
        # Plot elements
        self.scan_plot = self.ax1.scatter([], [], s=2, c='gray', label='Scan Points')
        self.robot_circle = self.ax1.add_patch(Circle((0, 0), 0.5, color='blue', alpha=0.5, label='Robot'))
        
        # Add velocity vectors
        self.nominal_arrow = None
        self.corrected_arrow = None
        
        # Add legend
        self.ax1.legend(loc='upper right')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add status panel
        self.status_frame = tk.Frame(self.root)
        self.status_frame.pack(fill=tk.X)
        
        self.nominal_label = tk.Label(self.status_frame, text="Nominal: v=0.0, ω=0.0")
        self.nominal_label.pack(side=tk.LEFT, padx=10)
        
        self.corrected_label = tk.Label(self.status_frame, text="Corrected: v=0.0, ω=0.0")
        self.corrected_label.pack(side=tk.LEFT, padx=10)
        
        self.diff_label = tk.Label(self.status_frame, text="Difference: Δv=0.0, Δω=0.0")
        self.diff_label.pack(side=tk.LEFT, padx=10)
        
        # Setup animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        
        # Add ROS 2 spinner timer
        self.root.after(10, self.spin_once)
    
    def update_plot(self, frame):
        # Update scan points
        all_points = []
        for topic, points in self.scan_data.items():
            all_points.extend(points)
        
        if all_points:
            x, y = zip(*all_points)
            self.scan_plot.set_offsets(np.column_stack((x, y)))
        
        # Update velocity vectors
        self.update_velocity_vectors()
        
        # Update status labels
        self.update_status_labels()
        
        return self.scan_plot,
    
    def update_velocity_vectors(self):
        # Remove previous arrows
        if self.nominal_arrow:
            self.nominal_arrow.remove()
        if self.corrected_arrow:
            self.corrected_arrow.remove()
        
        # Calculate nominal velocity vector
        nominal_scale = 3.0  # Scale factor for visualization
        nominal_x = self.nominal_cmd.linear.x * nominal_scale
        nominal_y = 0.0
        
        # Apply rotation based on angular velocity
        # This is a simplification - in reality, angular velocity would cause the robot to turn
        if abs(self.nominal_cmd.angular.z) > 0.001:
            nominal_angle = math.atan2(self.nominal_cmd.angular.z, self.nominal_cmd.linear.x)
            nominal_x = nominal_scale * self.nominal_cmd.linear.x * math.cos(nominal_angle)
            nominal_y = nominal_scale * self.nominal_cmd.linear.x * math.sin(nominal_angle)
        
        # Calculate corrected velocity vector
        corrected_scale = 3.0  # Scale factor for visualization
        corrected_x = self.corrected_cmd.linear.x * corrected_scale
        corrected_y = 0.0
        
        # Apply rotation based on angular velocity
        if abs(self.corrected_cmd.angular.z) > 0.001:
            corrected_angle = math.atan2(self.corrected_cmd.angular.z, self.corrected_cmd.linear.x)
            corrected_x = corrected_scale * self.corrected_cmd.linear.x * math.cos(corrected_angle)
            corrected_y = corrected_scale * self.corrected_cmd.linear.x * math.sin(corrected_angle)
        
        # Draw new arrows
        self.nominal_arrow = self.ax1.arrow(0, 0, nominal_x, nominal_y, 
                                           head_width=0.3, head_length=0.5, 
                                           fc='blue', ec='blue', label='Nominal')
        
        self.corrected_arrow = self.ax1.arrow(0, 0, corrected_x, corrected_y, 
                                             head_width=0.3, head_length=0.5, 
                                             fc='red', ec='red', label='Corrected')
    
    def update_status_labels(self):
        # Update velocity labels
        self.nominal_label.config(
            text=f"Nominal: v={self.nominal_cmd.linear.x:.2f}, ω={self.nominal_cmd.angular.z:.2f}")
        
        self.corrected_label.config(
            text=f"Corrected: v={self.corrected_cmd.linear.x:.2f}, ω={self.corrected_cmd.angular.z:.2f}")
        
        # Calculate differences
        linear_diff = self.corrected_cmd.linear.x - self.nominal_cmd.linear.x
        angular_diff = self.corrected_cmd.angular.z - self.nominal_cmd.angular.z
        
        # Update difference label
        self.diff_label.config(
            text=f"Difference: Δv={linear_diff:.2f}, Δω={angular_diff:.2f}")
        
        # Highlight differences when significant
        if abs(linear_diff) > 0.05 or abs(angular_diff) > 0.05:
            self.diff_label.config(fg='red')
        else:
            self.diff_label.config(fg='black')
    
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(10, self.spin_once)

def main(args=None):
    rclpy.init(args=args)
    visualizer = RDSVisualization()
    
    try:
        visualizer.root.mainloop()
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()