#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
from exoskeleton_control.msg import JointsTrajectory

class TrajectoryPlotter:
    def __init__(self):
        rospy.init_node('trajectory_plotter_node', anonymous=True)
        
        # Buffer size for plotting (number of data points to keep)
        self.buffer_size = 500
        
        # Initialize data buffers
        self.time_buffer = deque(maxlen=self.buffer_size)
        self.rhip_pos_buffer = deque(maxlen=self.buffer_size)
        self.rknee_pos_buffer = deque(maxlen=self.buffer_size)
        self.rhip_vel_buffer = deque(maxlen=self.buffer_size)
        self.rknee_vel_buffer = deque(maxlen=self.buffer_size)
        
        # Initialize start time
        self.start_time = None
        
        # Subscribe to trajectory topic
        self.trajectory_sub = rospy.Subscriber(
            '/exoskeleton/joints_trajectory', 
            JointsTrajectory, 
            self.trajectory_callback
        )
        
        # Setup matplotlib
        plt.style.use('default')
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Real-time Joint Trajectory Visualization', fontsize=16)
        
        # Initialize plots
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=2, label='Right Hip Position')
        self.line2, = self.ax2.plot([], [], 'r-', linewidth=2, label='Right Knee Position')
        self.line3, = self.ax3.plot([], [], 'g-', linewidth=2, label='Right Hip Velocity')
        self.line4, = self.ax4.plot([], [], 'm-', linewidth=2, label='Right Knee Velocity')
        
        # Configure axes
        self.ax1.set_title('Right Hip Position (rad)')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Position (rad)')
        self.ax1.grid(True)
        self.ax1.legend()
        
        self.ax2.set_title('Right Knee Position (rad)')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Position (rad)')
        self.ax2.grid(True)
        self.ax2.legend()
        
        self.ax3.set_title('Right Hip Velocity (rad/s)')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Velocity (rad/s)')
        self.ax3.grid(True)
        self.ax3.legend()
        
        self.ax4.set_title('Right Knee Velocity (rad/s)')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Velocity (rad/s)')
        self.ax4.grid(True)
        self.ax4.legend()
        
        plt.tight_layout()
        
        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=50, blit=False, cache_frame_data=False
        )
        
        rospy.loginfo("Trajectory plotter node initialized. Waiting for trajectory data...")
    
    def trajectory_callback(self, msg):
        current_time = rospy.get_time()
        
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time
        
        # Store data in buffers
        self.time_buffer.append(relative_time)
        self.rhip_pos_buffer.append(msg.Rhip_pos_ref)
        self.rknee_pos_buffer.append(msg.Rknee_pos_ref)
        self.rhip_vel_buffer.append(msg.Rhip_vel_ref)
        self.rknee_vel_buffer.append(msg.Rknee_vel_ref)
    
    def animate(self, frame):
        if len(self.time_buffer) < 2:
            return self.line1, self.line2, self.line3, self.line4
        
        # Convert deques to numpy arrays for plotting
        time_data = np.array(self.time_buffer)
        rhip_pos_data = np.array(self.rhip_pos_buffer)
        rknee_pos_data = np.array(self.rknee_pos_buffer)
        rhip_vel_data = np.array(self.rhip_vel_buffer)
        rknee_vel_data = np.array(self.rknee_vel_buffer)
        
        # Update line data
        self.line1.set_data(time_data, rhip_pos_data)
        self.line2.set_data(time_data, rknee_pos_data)
        self.line3.set_data(time_data, rhip_vel_data)
        self.line4.set_data(time_data, rknee_vel_data)
        
        # Auto-scale axes
        if len(time_data) > 0:
            time_min, time_max = time_data.min(), time_data.max()
            time_range = max(time_max - time_min, 1.0)
            
            # Position plots
            rhip_pos_min, rhip_pos_max = rhip_pos_data.min(), rhip_pos_data.max()
            rhip_pos_range = max(rhip_pos_max - rhip_pos_min, 0.1)
            self.ax1.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax1.set_ylim(rhip_pos_min - 0.1 * rhip_pos_range, rhip_pos_max + 0.1 * rhip_pos_range)
            
            rknee_pos_min, rknee_pos_max = rknee_pos_data.min(), rknee_pos_data.max()
            rknee_pos_range = max(rknee_pos_max - rknee_pos_min, 0.1)
            self.ax2.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax2.set_ylim(rknee_pos_min - 0.1 * rknee_pos_range, rknee_pos_max + 0.1 * rknee_pos_range)
            
            # Velocity plots
            rhip_vel_min, rhip_vel_max = rhip_vel_data.min(), rhip_vel_data.max()
            rhip_vel_range = max(rhip_vel_max - rhip_vel_min, 0.1)
            self.ax3.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax3.set_ylim(rhip_vel_min - 0.1 * rhip_vel_range, rhip_vel_max + 0.1 * rhip_vel_range)
            
            rknee_vel_min, rknee_vel_max = rknee_vel_data.min(), rknee_vel_data.max()
            rknee_vel_range = max(rknee_vel_max - rknee_vel_min, 0.1)
            self.ax4.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax4.set_ylim(rknee_vel_min - 0.1 * rknee_vel_range, rknee_vel_max + 0.1 * rknee_vel_range)
        
        return self.line1, self.line2, self.line3, self.line4
    
    def run(self):
        try:
            plt.show()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down trajectory plotter...")
        finally:
            plt.close('all')

if __name__ == '__main__':
    try:
        plotter = TrajectoryPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass