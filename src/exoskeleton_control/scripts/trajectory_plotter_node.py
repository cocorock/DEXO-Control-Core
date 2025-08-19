#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import math
import os
import pickle
from datetime import datetime
from scipy.io import savemat
from exoskeleton_control.msg import JointsTrajectory, Torques, ExoskeletonState, EStopTrigger

class SystemPlotter:
    def __init__(self):
        rospy.init_node('system_plotter_node', anonymous=True)
        
        # Buffer size for plotting (number of data points to keep)
        # 5 seconds at 200Hz = 1000 data points
        self.buffer_size = 1000
        
        # Initialize data buffers
        self.time_buffer = deque(maxlen=self.buffer_size)
        self.rhip_pos_buffer = deque(maxlen=self.buffer_size)
        self.rknee_pos_buffer = deque(maxlen=self.buffer_size)
        self.rhip_vel_buffer = deque(maxlen=self.buffer_size)
        self.rknee_vel_buffer = deque(maxlen=self.buffer_size)
        self.rhip_ff_torque_buffer = deque(maxlen=self.buffer_size)
        self.rknee_ff_torque_buffer = deque(maxlen=self.buffer_size)
        self.rhip_motor_torque_buffer = deque(maxlen=self.buffer_size)
        self.rknee_motor_torque_buffer = deque(maxlen=self.buffer_size)
        
        # Current state buffers with timestamps
        self.rhip_pos_current_buffer = deque(maxlen=self.buffer_size)
        self.rknee_pos_current_buffer = deque(maxlen=self.buffer_size)
        self.rhip_vel_current_buffer = deque(maxlen=self.buffer_size)
        self.rknee_vel_current_buffer = deque(maxlen=self.buffer_size)
        self.state_time_buffer = deque(maxlen=self.buffer_size)
        
        # Torque time buffer
        self.torque_time_buffer = deque(maxlen=self.buffer_size)
        
        # Initialize start time
        self.start_time = None
        
        # Subscribe to trajectory, torques, and state topics
        self.trajectory_sub = rospy.Subscriber(
            '/exoskeleton/joints_trajectory', 
            JointsTrajectory, 
            self.trajectory_callback
        )
        self.torques_sub = rospy.Subscriber(
            '/exoskeleton/torques', 
            Torques, 
            self.torques_callback
        )
        self.state_sub = rospy.Subscriber(
            '/exoskeleton/state', 
            ExoskeletonState, 
            self.state_callback
        )
        self.emergency_stop_sub = rospy.Subscriber(
            '/exoskeleton/emergency_stop',
            EStopTrigger,
            self.emergency_stop_callback
        )
        
        # Setup matplotlib with 2x3 grid and black background
        plt.style.use('dark_background')
        self.fig, ((self.ax1, self.ax2, self.ax3), (self.ax4, self.ax5, self.ax6)) = plt.subplots(2, 3, figsize=(18, 10))
        self.fig.patch.set_facecolor('black')
        self.fig.suptitle('Real-time Exoskeleton System Visualization', fontsize=16, color='white')
        
        # Initialize plots - swapped positions: 1,3,5 (top row), 2,4,6 (bottom row)
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=1, label='Right Hip Position Ref')
        self.line1_current, = self.ax1.plot([], [], 'b--', linewidth=1, label='Right Hip Position Current')
        self.line3, = self.ax2.plot([], [], 'g-', linewidth=1, label='Right Hip Velocity Ref')
        self.line3_current, = self.ax2.plot([], [], 'g--', linewidth=1, label='Right Hip Velocity Current')
        self.line5, = self.ax3.plot([], [], 'c-', linewidth=1, label='Hip FF Torque')
        self.line5b, = self.ax3.plot([], [], 'orange', linewidth=1, linestyle='--', label='Hip Motor Torque')
        self.line2, = self.ax4.plot([], [], 'r-', linewidth=1, label='Right Knee Position Ref')
        self.line2_current, = self.ax4.plot([], [], 'r--', linewidth=1, label='Right Knee Position Current')
        self.line4, = self.ax5.plot([], [], 'm-', linewidth=1, label='Right Knee Velocity Ref')
        self.line4_current, = self.ax5.plot([], [], 'm--', linewidth=1, label='Right Knee Velocity Current')
        self.line6, = self.ax6.plot([], [], 'y-', linewidth=1, label='Knee FF Torque')
        self.line6b, = self.ax6.plot([], [], 'purple', linewidth=1, linestyle='--', label='Knee Motor Torque')
        
        # Configure axes with dark theme - updated for swapped layout
        axes = [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]
        titles = ['Right Hip Position (deg)', 'Right Hip Velocity (deg/s)', 'Right Hip Torques (N⋅m)',
                 'Right Knee Position (deg)', 'Right Knee Velocity (deg/s)', 'Right Knee Torques (N⋅m)']
        ylabels = ['Position (deg)', 'Velocity (deg/s)', 'Torque (N⋅m)', 
                  'Position (deg)', 'Velocity (deg/s)', 'Torque (N⋅m)']
        
        for i, ax in enumerate(axes):
            ax.set_facecolor('black')
            ax.set_title(titles[i], color='white')
            ax.set_xlabel('Time (s)', color='white')
            ax.set_ylabel(ylabels[i], color='white')
            ax.grid(True, color='gray', alpha=0.3)
            ax.tick_params(colors='white')
            # Create legend with compatible parameters
            legend = ax.legend(facecolor='black', edgecolor='white')
            # Set legend text color manually for compatibility
            for text in legend.get_texts():
                text.set_color('white')
        
        plt.tight_layout()
        
        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=50, blit=False, cache_frame_data=False
        )
        
        rospy.loginfo("System plotter node initialized. Waiting for trajectory, torque, and state data...")
    
    def trajectory_callback(self, msg):
        current_time = rospy.get_time()
        
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time
        
        # Store data in buffers (convert positions and velocities to degrees)
        self.time_buffer.append(relative_time)
        self.rhip_pos_buffer.append(math.degrees(msg.Rhip_pos_ref))
        self.rknee_pos_buffer.append(math.degrees(msg.Rknee_pos_ref))
        self.rhip_vel_buffer.append(math.degrees(msg.Rhip_vel_ref))
        self.rknee_vel_buffer.append(math.degrees(msg.Rknee_vel_ref))
    
    def state_callback(self, msg):
        """Process current state data from motor control node."""
        current_time = rospy.get_time()
        
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time
        
        # Store state data with synchronized timestamps
        self.state_time_buffer.append(relative_time)
        self.rhip_pos_current_buffer.append(math.degrees(msg.Rhip_pos_st))
        self.rknee_pos_current_buffer.append(math.degrees(msg.Rknee_pos_st))
        self.rhip_vel_current_buffer.append(math.degrees(msg.Rhip_vel_st))
        self.rknee_vel_current_buffer.append(math.degrees(msg.Rknee_vel_st))
    
    def torques_callback(self, msg):
        """Process torque data from motor control node."""
        current_time = rospy.get_time()
        
        if self.start_time is None:
            self.start_time = current_time
        
        relative_time = current_time - self.start_time
        
        if len(msg.torques) >= 4:
            # Store torque data with synchronized timestamps
            self.torque_time_buffer.append(relative_time)
            # msg.torques format: [R_hip_ff, R_knee_ff, R_hip_motor, R_knee_motor]
            self.rhip_ff_torque_buffer.append(msg.torques[0])
            self.rknee_ff_torque_buffer.append(msg.torques[1])
            self.rhip_motor_torque_buffer.append(msg.torques[2])
            self.rknee_motor_torque_buffer.append(msg.torques[3])
    
    def emergency_stop_callback(self, msg):
        """Handle emergency stop trigger and save current plot data."""
        if msg.trigger:
            rospy.logwarn("Emergency stop triggered - saving plot data...")
            self.save_emergency_data()
            rospy.logwarn("Emergency data saved successfully")
    
    def save_emergency_data(self):
        """Save the last 5 seconds of plot data to file."""
        try:
            # Create emergency data directory if it doesn't exist
            emergency_dir = os.path.expanduser("~/emergency_data")
            if not os.path.exists(emergency_dir):
                os.makedirs(emergency_dir)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            mat_filename = os.path.join(emergency_dir, f"emergency_plot_data_{timestamp}.mat")
            pkl_filename = os.path.join(emergency_dir, f"emergency_plot_data_{timestamp}.pkl")
            
            # Collect all current buffer data for MATLAB
            matlab_data = {
                'timestamp': timestamp,
                'buffer_size': float(self.buffer_size),
                'trajectory_time': np.array(list(self.time_buffer)),
                'rhip_pos_ref': np.array(list(self.rhip_pos_buffer)),
                'rknee_pos_ref': np.array(list(self.rknee_pos_buffer)),
                'rhip_vel_ref': np.array(list(self.rhip_vel_buffer)),
                'rknee_vel_ref': np.array(list(self.rknee_vel_buffer)),
                'state_time': np.array(list(self.state_time_buffer)),
                'rhip_pos_current': np.array(list(self.rhip_pos_current_buffer)),
                'rknee_pos_current': np.array(list(self.rknee_pos_current_buffer)),
                'rhip_vel_current': np.array(list(self.rhip_vel_current_buffer)),
                'rknee_vel_current': np.array(list(self.rknee_vel_current_buffer)),
                'torque_time': np.array(list(self.torque_time_buffer)),
                'rhip_ff_torque': np.array(list(self.rhip_ff_torque_buffer)),
                'rknee_ff_torque': np.array(list(self.rknee_ff_torque_buffer)),
                'rhip_motor_torque': np.array(list(self.rhip_motor_torque_buffer)),
                'rknee_motor_torque': np.array(list(self.rknee_motor_torque_buffer))
            }
            
            # Collect all current buffer data for pickle backup
            emergency_data = {
                'timestamp': timestamp,
                'buffer_size': self.buffer_size,
                'trajectory_data': {
                    'time': list(self.time_buffer),
                    'rhip_pos_ref': list(self.rhip_pos_buffer),
                    'rknee_pos_ref': list(self.rknee_pos_buffer),
                    'rhip_vel_ref': list(self.rhip_vel_buffer),
                    'rknee_vel_ref': list(self.rknee_vel_buffer)
                },
                'state_data': {
                    'time': list(self.state_time_buffer),
                    'rhip_pos_current': list(self.rhip_pos_current_buffer),
                    'rknee_pos_current': list(self.rknee_pos_current_buffer),
                    'rhip_vel_current': list(self.rhip_vel_current_buffer),
                    'rknee_vel_current': list(self.rknee_vel_current_buffer)
                },
                'torque_data': {
                    'time': list(self.torque_time_buffer),
                    'rhip_ff_torque': list(self.rhip_ff_torque_buffer),
                    'rknee_ff_torque': list(self.rknee_ff_torque_buffer),
                    'rhip_motor_torque': list(self.rhip_motor_torque_buffer),
                    'rknee_motor_torque': list(self.rknee_motor_torque_buffer)
                },
                'plot_info': {
                    'plot_names': ['Right Hip Position', 'Right Hip Velocity', 'Right Hip Torques',
                                  'Right Knee Position', 'Right Knee Velocity', 'Right Knee Torques'],
                    'units': ['deg', 'deg/s', 'N⋅m', 'deg', 'deg/s', 'N⋅m']
                }
            }
            
            # Save data to MATLAB file
            savemat(mat_filename, matlab_data)
            rospy.logwarn(f"Emergency plot data saved to MATLAB file: {mat_filename}")
            
            # Save backup pickle file
            with open(pkl_filename, 'wb') as f:
                pickle.dump(emergency_data, f)
            rospy.logwarn(f"Emergency plot data backup saved to: {pkl_filename}")
            
        except Exception as e:
            rospy.logerr(f"Failed to save emergency data: {str(e)}")
    
    def animate(self, frame):
        if len(self.time_buffer) < 2:
            return (self.line1, self.line1_current, self.line2, self.line2_current, 
                   self.line3, self.line3_current, self.line4, self.line4_current, 
                   self.line5, self.line5b, self.line6, self.line6b)
        
        # Convert deques to numpy arrays for plotting
        time_data = np.array(self.time_buffer)
        rhip_pos_data = np.array(self.rhip_pos_buffer)
        rknee_pos_data = np.array(self.rknee_pos_buffer)
        rhip_vel_data = np.array(self.rhip_vel_buffer)
        rknee_vel_data = np.array(self.rknee_vel_buffer)
        
        # Current state data with independent timestamps
        if len(self.state_time_buffer) > 0:
            state_time_data = np.array(self.state_time_buffer)
            rhip_pos_current_data = np.array(self.rhip_pos_current_buffer)
            rknee_pos_current_data = np.array(self.rknee_pos_current_buffer)
            rhip_vel_current_data = np.array(self.rhip_vel_current_buffer)
            rknee_vel_current_data = np.array(self.rknee_vel_current_buffer)
        else:
            state_time_data = np.array([])
            rhip_pos_current_data = np.array([])
            rknee_pos_current_data = np.array([])
            rhip_vel_current_data = np.array([])
            rknee_vel_current_data = np.array([])
        
        # Torque data with independent timestamps
        if len(self.torque_time_buffer) > 0:
            torque_time_data = np.array(self.torque_time_buffer)
            rhip_ff_torque_data = np.array(self.rhip_ff_torque_buffer)
            rknee_ff_torque_data = np.array(self.rknee_ff_torque_buffer)
            rhip_motor_torque_data = np.array(self.rhip_motor_torque_buffer)
            rknee_motor_torque_data = np.array(self.rknee_motor_torque_buffer)
        else:
            torque_time_data = np.array([])
            rhip_ff_torque_data = np.array([])
            rknee_ff_torque_data = np.array([])
            rhip_motor_torque_data = np.array([])
            rknee_motor_torque_data = np.array([])
        
        # Update line data (positions swapped to match new layout)
        self.line1.set_data(time_data, rhip_pos_data)  # ax1: Hip Position
        self.line1_current.set_data(state_time_data, rhip_pos_current_data)
        self.line3.set_data(time_data, rhip_vel_data)  # ax2: Hip Velocity  
        self.line3_current.set_data(state_time_data, rhip_vel_current_data)
        self.line5.set_data(torque_time_data, rhip_ff_torque_data)  # ax3: Hip Torques
        self.line5b.set_data(torque_time_data, rhip_motor_torque_data)
        self.line2.set_data(time_data, rknee_pos_data)  # ax4: Knee Position
        self.line2_current.set_data(state_time_data, rknee_pos_current_data)
        self.line4.set_data(time_data, rknee_vel_data)  # ax5: Knee Velocity
        self.line4_current.set_data(state_time_data, rknee_vel_current_data)
        self.line6.set_data(torque_time_data, rknee_ff_torque_data)  # ax6: Knee Torques
        self.line6b.set_data(torque_time_data, rknee_motor_torque_data)
        
        # Auto-scale axes
        if len(time_data) > 0:
            time_min, time_max = time_data.min(), time_data.max()
            time_range = max(time_max - time_min, 1.0)
            
            # Hip position plot (ax1)
            rhip_pos_combined = np.concatenate([rhip_pos_data, rhip_pos_current_data]) if len(rhip_pos_current_data) > 0 else rhip_pos_data
            rhip_pos_min, rhip_pos_max = rhip_pos_combined.min(), rhip_pos_combined.max()
            rhip_pos_range = max(rhip_pos_max - rhip_pos_min, 1.0)  # Use 1 degree minimum range
            self.ax1.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax1.set_ylim(rhip_pos_min - 0.1 * rhip_pos_range, rhip_pos_max + 0.1 * rhip_pos_range)
            
            # Hip velocity plot (ax2) 
            rhip_vel_combined = np.concatenate([rhip_vel_data, rhip_vel_current_data]) if len(rhip_vel_current_data) > 0 else rhip_vel_data
            rhip_vel_min, rhip_vel_max = rhip_vel_combined.min(), rhip_vel_combined.max()
            rhip_vel_range = max(rhip_vel_max - rhip_vel_min, 1.0)  # Use 1 deg/s minimum range
            self.ax2.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax2.set_ylim(rhip_vel_min - 0.1 * rhip_vel_range, rhip_vel_max + 0.1 * rhip_vel_range)
            
            # Knee position plot (ax4)
            rknee_pos_combined = np.concatenate([rknee_pos_data, rknee_pos_current_data]) if len(rknee_pos_current_data) > 0 else rknee_pos_data
            rknee_pos_min, rknee_pos_max = rknee_pos_combined.min(), rknee_pos_combined.max()
            rknee_pos_range = max(rknee_pos_max - rknee_pos_min, 1.0)  # Use 1 degree minimum range
            self.ax4.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax4.set_ylim(rknee_pos_min - 0.1 * rknee_pos_range, rknee_pos_max + 0.1 * rknee_pos_range)
            
            # Knee velocity plot (ax5)
            rknee_vel_combined = np.concatenate([rknee_vel_data, rknee_vel_current_data]) if len(rknee_vel_current_data) > 0 else rknee_vel_data
            rknee_vel_min, rknee_vel_max = rknee_vel_combined.min(), rknee_vel_combined.max()
            rknee_vel_range = max(rknee_vel_max - rknee_vel_min, 1.0)  # Use 1 deg/s minimum range
            self.ax5.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax5.set_ylim(rknee_vel_min - 0.1 * rknee_vel_range, rknee_vel_max + 0.1 * rknee_vel_range)
            
            # Torque plots auto-scaling
            if len(torque_time_data) > 0:
                torque_time_min, torque_time_max = torque_time_data.min(), torque_time_data.max()
                torque_time_range = max(torque_time_max - torque_time_min, 1.0)
                
                # Hip torques (ax3)
                if len(rhip_ff_torque_data) > 0 and len(rhip_motor_torque_data) > 0:
                    hip_torque_combined = np.concatenate([rhip_ff_torque_data, rhip_motor_torque_data])
                    hip_torque_min, hip_torque_max = hip_torque_combined.min(), hip_torque_combined.max()
                    hip_torque_range = max(hip_torque_max - hip_torque_min, 0.1)
                    self.ax3.set_xlim(torque_time_min - 0.1 * torque_time_range, torque_time_max + 0.1 * torque_time_range)
                    self.ax3.set_ylim(hip_torque_min - 0.1 * hip_torque_range, hip_torque_max + 0.1 * hip_torque_range)
                
                # Knee torques (ax6)
                if len(rknee_ff_torque_data) > 0 and len(rknee_motor_torque_data) > 0:
                    knee_torque_combined = np.concatenate([rknee_ff_torque_data, rknee_motor_torque_data])
                    knee_torque_min, knee_torque_max = knee_torque_combined.min(), knee_torque_combined.max()
                    knee_torque_range = max(knee_torque_max - knee_torque_min, 0.1)
                    self.ax6.set_xlim(torque_time_min - 0.1 * torque_time_range, torque_time_max + 0.1 * torque_time_range)
                    self.ax6.set_ylim(knee_torque_min - 0.1 * knee_torque_range, knee_torque_max + 0.1 * knee_torque_range)
        
        return (self.line1, self.line1_current, self.line2, self.line2_current, 
               self.line3, self.line3_current, self.line4, self.line4_current, 
               self.line5, self.line5b, self.line6, self.line6b)
    
    def run(self):
        try:
            plt.show()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down system plotter...")
        finally:
            plt.close('all')

if __name__ == '__main__':
    try:
        plotter = SystemPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass