#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
from exoskeleton_control.msg import JointsTrajectory, Torques, ExoskeletonState

class SystemPlotter:
    def __init__(self):
        rospy.init_node('system_plotter_node', anonymous=True)
        
        # Buffer size for plotting (number of data points to keep)
        self.buffer_size = 500
        
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
        
        # Current state buffers
        self.rhip_pos_current_buffer = deque(maxlen=self.buffer_size)
        self.rknee_pos_current_buffer = deque(maxlen=self.buffer_size)
        self.rhip_vel_current_buffer = deque(maxlen=self.buffer_size)
        self.rknee_vel_current_buffer = deque(maxlen=self.buffer_size)
        
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
        
        # Setup matplotlib with 2x3 grid
        plt.style.use('default')
        self.fig, ((self.ax1, self.ax2, self.ax3), (self.ax4, self.ax5, self.ax6)) = plt.subplots(2, 3, figsize=(18, 10))
        self.fig.suptitle('Real-time Exoskeleton System Visualization', fontsize=16)
        
        # Initialize plots
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=1, label='Right Hip Position Ref')
        self.line1_current, = self.ax1.plot([], [], 'b--', linewidth=1, label='Right Hip Position Current')
        self.line2, = self.ax2.plot([], [], 'r-', linewidth=1, label='Right Knee Position Ref')
        self.line2_current, = self.ax2.plot([], [], 'r--', linewidth=1, label='Right Knee Position Current')
        self.line3, = self.ax3.plot([], [], 'g-', linewidth=1, label='Right Hip Velocity Ref')
        self.line3_current, = self.ax3.plot([], [], 'g--', linewidth=1, label='Right Hip Velocity Current')
        self.line4, = self.ax4.plot([], [], 'm-', linewidth=1, label='Right Knee Velocity Ref')
        self.line4_current, = self.ax4.plot([], [], 'm--', linewidth=1, label='Right Knee Velocity Current')
        self.line5, = self.ax5.plot([], [], 'c-', linewidth=1, label='Hip FF Torque')
        self.line5b, = self.ax5.plot([], [], 'orange', linewidth=1, linestyle='--', label='Hip Motor Torque')
        self.line6, = self.ax6.plot([], [], 'y-', linewidth=1, label='Knee FF Torque')
        self.line6b, = self.ax6.plot([], [], 'purple', linewidth=1, linestyle='--', label='Knee Motor Torque')
        
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
        
        self.ax5.set_title('Right Hip Torques (N⋅m)')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Torque (N⋅m)')
        self.ax5.grid(True)
        self.ax5.legend()
        
        self.ax6.set_title('Right Knee Torques (N⋅m)')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Torque (N⋅m)')
        self.ax6.grid(True)
        self.ax6.legend()
        
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
        
        # Store data in buffers
        self.time_buffer.append(relative_time)
        self.rhip_pos_buffer.append(msg.Rhip_pos_ref)
        self.rknee_pos_buffer.append(msg.Rknee_pos_ref)
        self.rhip_vel_buffer.append(msg.Rhip_vel_ref)
        self.rknee_vel_buffer.append(msg.Rknee_vel_ref)
    
    def state_callback(self, msg):
        """Process current state data from motor control node."""
        # Only store current state if we have trajectory data (same time buffer)
        if len(self.time_buffer) > 0:
            self.rhip_pos_current_buffer.append(msg.Rhip_pos_st)
            self.rknee_pos_current_buffer.append(msg.Rknee_pos_st)
            self.rhip_vel_current_buffer.append(msg.Rhip_vel_st)
            self.rknee_vel_current_buffer.append(msg.Rknee_vel_st)
    
    def torques_callback(self, msg):
        """Process torque data from motor control node."""
        if len(msg.torques) >= 4:
            # msg.torques format: [R_hip_ff, R_knee_ff, R_hip_motor, R_knee_motor]
            self.rhip_ff_torque_buffer.append(msg.torques[0])
            self.rknee_ff_torque_buffer.append(msg.torques[1])
            self.rhip_motor_torque_buffer.append(msg.torques[2])
            self.rknee_motor_torque_buffer.append(msg.torques[3])
    
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
        
        # Current state data (pad with zeros if not enough data)
        state_min_len = min(len(self.time_buffer), len(self.rhip_pos_current_buffer))
        if state_min_len > 0:
            rhip_pos_current_data = np.array(list(self.rhip_pos_current_buffer)[-state_min_len:])
            rknee_pos_current_data = np.array(list(self.rknee_pos_current_buffer)[-state_min_len:])
            rhip_vel_current_data = np.array(list(self.rhip_vel_current_buffer)[-state_min_len:])
            rknee_vel_current_data = np.array(list(self.rknee_vel_current_buffer)[-state_min_len:])
            state_time_data = time_data[-state_min_len:]
        else:
            rhip_pos_current_data = np.array([])
            rknee_pos_current_data = np.array([])
            rhip_vel_current_data = np.array([])
            rknee_vel_current_data = np.array([])
            state_time_data = np.array([])
        
        # Torque data (pad with zeros if not enough data)
        min_len = min(len(self.time_buffer), len(self.rhip_ff_torque_buffer))
        if min_len > 0:
            rhip_ff_torque_data = np.array(list(self.rhip_ff_torque_buffer)[-min_len:])
            rknee_ff_torque_data = np.array(list(self.rknee_ff_torque_buffer)[-min_len:])
            rhip_motor_torque_data = np.array(list(self.rhip_motor_torque_buffer)[-min_len:])
            rknee_motor_torque_data = np.array(list(self.rknee_motor_torque_buffer)[-min_len:])
            torque_time_data = time_data[-min_len:]
        else:
            rhip_ff_torque_data = np.array([])
            rknee_ff_torque_data = np.array([])
            rhip_motor_torque_data = np.array([])
            rknee_motor_torque_data = np.array([])
            torque_time_data = np.array([])
        
        # Update line data
        self.line1.set_data(time_data, rhip_pos_data)
        self.line1_current.set_data(state_time_data, rhip_pos_current_data)
        self.line2.set_data(time_data, rknee_pos_data)
        self.line2_current.set_data(state_time_data, rknee_pos_current_data)
        self.line3.set_data(time_data, rhip_vel_data)
        self.line3_current.set_data(state_time_data, rhip_vel_current_data)
        self.line4.set_data(time_data, rknee_vel_data)
        self.line4_current.set_data(state_time_data, rknee_vel_current_data)
        self.line5.set_data(torque_time_data, rhip_ff_torque_data)
        self.line5b.set_data(torque_time_data, rhip_motor_torque_data)
        self.line6.set_data(torque_time_data, rknee_ff_torque_data)
        self.line6b.set_data(torque_time_data, rknee_motor_torque_data)
        
        # Auto-scale axes
        if len(time_data) > 0:
            time_min, time_max = time_data.min(), time_data.max()
            time_range = max(time_max - time_min, 1.0)
            
            # Position plots (include both reference and current data for scaling)
            rhip_pos_combined = np.concatenate([rhip_pos_data, rhip_pos_current_data]) if len(rhip_pos_current_data) > 0 else rhip_pos_data
            rhip_pos_min, rhip_pos_max = rhip_pos_combined.min(), rhip_pos_combined.max()
            rhip_pos_range = max(rhip_pos_max - rhip_pos_min, 0.1)
            self.ax1.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax1.set_ylim(rhip_pos_min - 0.1 * rhip_pos_range, rhip_pos_max + 0.1 * rhip_pos_range)
            
            rknee_pos_combined = np.concatenate([rknee_pos_data, rknee_pos_current_data]) if len(rknee_pos_current_data) > 0 else rknee_pos_data
            rknee_pos_min, rknee_pos_max = rknee_pos_combined.min(), rknee_pos_combined.max()
            rknee_pos_range = max(rknee_pos_max - rknee_pos_min, 0.1)
            self.ax2.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax2.set_ylim(rknee_pos_min - 0.1 * rknee_pos_range, rknee_pos_max + 0.1 * rknee_pos_range)
            
            # Velocity plots (include both reference and current data for scaling)
            rhip_vel_combined = np.concatenate([rhip_vel_data, rhip_vel_current_data]) if len(rhip_vel_current_data) > 0 else rhip_vel_data
            rhip_vel_min, rhip_vel_max = rhip_vel_combined.min(), rhip_vel_combined.max()
            rhip_vel_range = max(rhip_vel_max - rhip_vel_min, 0.1)
            self.ax3.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax3.set_ylim(rhip_vel_min - 0.1 * rhip_vel_range, rhip_vel_max + 0.1 * rhip_vel_range)
            
            rknee_vel_combined = np.concatenate([rknee_vel_data, rknee_vel_current_data]) if len(rknee_vel_current_data) > 0 else rknee_vel_data
            rknee_vel_min, rknee_vel_max = rknee_vel_combined.min(), rknee_vel_combined.max()
            rknee_vel_range = max(rknee_vel_max - rknee_vel_min, 0.1)
            self.ax4.set_xlim(time_min - 0.1 * time_range, time_max + 0.1 * time_range)
            self.ax4.set_ylim(rknee_vel_min - 0.1 * rknee_vel_range, rknee_vel_max + 0.1 * rknee_vel_range)
            
            # Torque plots auto-scaling
            if len(torque_time_data) > 0:
                torque_time_min, torque_time_max = torque_time_data.min(), torque_time_data.max()
                torque_time_range = max(torque_time_max - torque_time_min, 1.0)
                
                # Hip torques
                if len(rhip_ff_torque_data) > 0 and len(rhip_motor_torque_data) > 0:
                    hip_torque_combined = np.concatenate([rhip_ff_torque_data, rhip_motor_torque_data])
                    hip_torque_min, hip_torque_max = hip_torque_combined.min(), hip_torque_combined.max()
                    hip_torque_range = max(hip_torque_max - hip_torque_min, 0.1)
                    self.ax5.set_xlim(torque_time_min - 0.1 * torque_time_range, torque_time_max + 0.1 * torque_time_range)
                    self.ax5.set_ylim(hip_torque_min - 0.1 * hip_torque_range, hip_torque_max + 0.1 * hip_torque_range)
                
                # Knee torques
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