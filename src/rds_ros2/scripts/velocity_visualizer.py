#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from threading import Lock
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class VelocityVisualizer(Node):
    def __init__(self):
        super().__init__('velocity_visualizer')
        
        # Create subscriptions for nominal and corrected velocities
        self.nominal_sub = self.create_subscription(
            Twist, '/remote_cmd_vel', self.nominal_callback, 10)
        
        self.corrected_sub = self.create_subscription(
            Twist, '/rds_modulated_cmd_vel', self.corrected_callback, 10)
        
        # Store latest velocities
        self.nominal_linear = 0.0
        self.nominal_angular = 0.0
        self.corrected_linear = 0.0
        self.corrected_angular = 0.0
        
        # Thread safety
        self.lock = Lock()
        
        # History for plotting
        self.max_history = 100
        self.time_history = np.arange(-self.max_history + 1, 1)
        self.nominal_linear_history = np.zeros(self.max_history)
        self.nominal_angular_history = np.zeros(self.max_history)
        self.corrected_linear_history = np.zeros(self.max_history)
        self.corrected_angular_history = np.zeros(self.max_history)
        
        # Setup the visualization
        self.setup_visualization()
        
        self.get_logger().info("Velocity visualizer started")
    
    def nominal_callback(self, msg):
        with self.lock:
            self.nominal_linear = msg.linear.x
            self.nominal_angular = msg.angular.z
            
            # Update history
            self.nominal_linear_history = np.roll(self.nominal_linear_history, -1)
            self.nominal_linear_history[-1] = self.nominal_linear
            
            self.nominal_angular_history = np.roll(self.nominal_angular_history, -1)
            self.nominal_angular_history[-1] = self.nominal_angular
    
    def corrected_callback(self, msg):
        with self.lock:
            self.corrected_linear = msg.linear.x
            self.corrected_angular = msg.angular.z
            
            # Update history
            self.corrected_linear_history = np.roll(self.corrected_linear_history, -1)
            self.corrected_linear_history[-1] = self.corrected_linear
            
            self.corrected_angular_history = np.roll(self.corrected_angular_history, -1)
            self.corrected_angular_history[-1] = self.corrected_angular
    
    def setup_visualization(self):
        # Create tkinter window
        self.root = tk.Tk()
        self.root.title("RDS Velocity Visualization")
        self.root.geometry("1000x800")
        
        # Create matplotlib figure
        self.fig = plt.figure(figsize=(10, 8))
        
        # Add velocity plots
        self.ax1 = self.fig.add_subplot(2, 2, 1)  # Linear velocity history
        self.ax2 = self.fig.add_subplot(2, 2, 2)  # Angular velocity history
        self.ax3 = self.fig.add_subplot(2, 2, (3, 4), polar=True)  # Velocity vector plot
        
        # Setup plots
        self.line_nominal_linear, = self.ax1.plot(self.time_history, self.nominal_linear_history, 
                                                 'b-', label='Nominal')
        self.line_corrected_linear, = self.ax1.plot(self.time_history, self.corrected_linear_history, 
                                                   'r-', label='Corrected')
        self.ax1.set_title("Linear Velocity (m/s)")
        self.ax1.set_xlabel("Time Steps")
        self.ax1.set_ylabel("Velocity")
        self.ax1.legend()
        self.ax1.grid(True)
        
        self.line_nominal_angular, = self.ax2.plot(self.time_history, self.nominal_angular_history, 
                                                  'b-', label='Nominal')
        self.line_corrected_angular, = self.ax2.plot(self.time_history, self.corrected_angular_history, 
                                                    'r-', label='Corrected')
        self.ax2.set_title("Angular Velocity (rad/s)")
        self.ax2.set_xlabel("Time Steps")
        self.ax2.set_ylabel("Velocity")
        self.ax2.legend()
        self.ax2.grid(True)
        
        # Setup vector plot
        self.ax3.set_title("Velocity Vectors")
        self.ax3.set_theta_zero_location("N")  # 0 is up
        self.ax3.set_theta_direction(-1)  # Clockwise
        
        # Add arrows for nominal and corrected velocities
        self.quiver_nominal = self.ax3.quiver(0, 0, 0, 0, color='blue', label='Nominal')
        self.quiver_corrected = self.ax3.quiver(0, 0, 0, 0, color='red', label='Corrected')
        self.ax3.set_rmax(1.5)  # Set maximum radius
        self.ax3.legend(loc='upper right')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Setup animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        
        # Add ROS 2 spinner timer
        self.root.after(10, self.spin_once)
    
    def update_plot(self, frame):
        with self.lock:
            # Update linear velocity plot
            self.line_nominal_linear.set_ydata(self.nominal_linear_history)
            self.line_corrected_linear.set_ydata(self.corrected_linear_history)
            
            # Update angular velocity plot
            self.line_nominal_angular.set_ydata(self.nominal_angular_history)
            self.line_corrected_angular.set_ydata(self.corrected_angular_history)
            
            # Auto-adjust y limits
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            # Calculate vector directions and magnitudes
            nominal_angle = np.arctan2(self.nominal_angular, self.nominal_linear)
            corrected_angle = np.arctan2(self.corrected_angular, self.corrected_linear)
            
            nominal_magnitude = np.sqrt(self.nominal_linear**2 + self.nominal_angular**2)
            corrected_magnitude = np.sqrt(self.corrected_linear**2 + self.corrected_angular**2)
            
            # Clear previous quivers
            self.ax3.clear()
            self.ax3.set_title("Velocity Vectors")
            self.ax3.set_theta_zero_location("N")
            self.ax3.set_theta_direction(-1)
            
            # Draw new quivers
            self.ax3.quiver(nominal_angle, 0, 0, nominal_magnitude, 
                           angles='xy', scale_units='xy', scale=1, color='blue', label='Nominal')
            self.ax3.quiver(corrected_angle, 0, 0, corrected_magnitude, 
                           angles='xy', scale_units='xy', scale=1, color='red', label='Corrected')
            
            self.ax3.set_rmax(1.5)
            self.ax3.legend(loc='upper right')
        
        return (self.line_nominal_linear, self.line_corrected_linear, 
                self.line_nominal_angular, self.line_corrected_angular)
    
    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(10, self.spin_once)

def main(args=None):
    rclpy.init(args=args)
    visualizer = VelocityVisualizer()
    
    try:
        visualizer.root.mainloop()
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()