#!/usr/bin/env python3

"""
Trajectory Monitor for TPGMM Testing
Monitors and displays real-time statistics about the generated trajectories.
"""

import rospy
import numpy as np
from exoskeleton_control.msg import JointsTrajectory, DualAnkleTrajectory

class TrajectoryMonitor:
    def __init__(self):
        rospy.init_node('trajectory_monitor')
        
        # Parameters
        self.update_rate = rospy.get_param('~update_rate', 10.0)  # Hz
        
        # Statistics tracking
        self.joint_stats = {
            'count': 0,
            'hip_pos': [],
            'knee_pos': [],
            'hip_vel': [],
            'knee_vel': []
        }
        
        self.ankle_stats = {
            'count': 0,
            'right_pos_x': [],
            'right_pos_y': [],
            'left_pos_x': [],
            'left_pos_y': [],
            'time_phases': []
        }
        
        # Subscribers
        rospy.Subscriber('joints_trajectory', JointsTrajectory, self.joints_callback)
        rospy.Subscriber('dual_ankle_trajectory', DualAnkleTrajectory, self.ankle_callback)
        
        # Statistics display timer
        self.stats_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.display_statistics)
        
        rospy.loginfo("Trajectory Monitor initialized")

    def joints_callback(self, msg):
        """Process joint trajectory messages"""
        self.joint_stats['count'] += 1
        self.joint_stats['hip_pos'].append(msg.Rhip_pos_ref)
        self.joint_stats['knee_pos'].append(msg.Rknee_pos_ref)
        self.joint_stats['hip_vel'].append(msg.Rhip_vel_ref)
        self.joint_stats['knee_vel'].append(msg.Rknee_vel_ref)
        
        # Keep only last 100 samples
        for key in ['hip_pos', 'knee_pos', 'hip_vel', 'knee_vel']:
            if len(self.joint_stats[key]) > 100:
                self.joint_stats[key] = self.joint_stats[key][-100:]

    def ankle_callback(self, msg):
        """Process dual ankle trajectory messages"""
        self.ankle_stats['count'] += 1
        self.ankle_stats['right_pos_x'].append(msg.right_ankle_pos_x)
        self.ankle_stats['right_pos_y'].append(msg.right_ankle_pos_y)
        self.ankle_stats['left_pos_x'].append(msg.left_ankle_pos_x)
        self.ankle_stats['left_pos_y'].append(msg.left_ankle_pos_y)
        self.ankle_stats['time_phases'].append(msg.time_phase)
        
        # Keep only last 100 samples
        for key in ['right_pos_x', 'right_pos_y', 'left_pos_x', 'left_pos_y', 'time_phases']:
            if len(self.ankle_stats[key]) > 100:
                self.ankle_stats[key] = self.ankle_stats[key][-100:]

    def display_statistics(self, event):
        """Display trajectory statistics"""
        if self.joint_stats['count'] == 0 and self.ankle_stats['count'] == 0:
            return
        
        print("\n" + "="*80)
        print("TPGMM TRAJECTORY MONITOR - REAL-TIME STATISTICS")
        print("="*80)
        
        # Joint trajectory statistics
        if self.joint_stats['count'] > 0:
            print(f"\nJOINT TRAJECTORIES (Total messages: {self.joint_stats['count']})")
            print("-" * 50)
            
            if len(self.joint_stats['hip_pos']) > 0:
                hip_pos_arr = np.array(self.joint_stats['hip_pos'])
                knee_pos_arr = np.array(self.joint_stats['knee_pos'])
                hip_vel_arr = np.array(self.joint_stats['hip_vel'])
                knee_vel_arr = np.array(self.joint_stats['knee_vel'])
                
                print(f"Hip Position  - Current: {hip_pos_arr[-1]:7.3f} rad ({np.degrees(hip_pos_arr[-1]):6.1f}°)")
                print(f"              - Range: [{np.min(hip_pos_arr):6.3f}, {np.max(hip_pos_arr):6.3f}] rad")
                print(f"              - Mean: {np.mean(hip_pos_arr):7.3f} rad, Std: {np.std(hip_pos_arr):6.3f}")
                
                print(f"Knee Position - Current: {knee_pos_arr[-1]:7.3f} rad ({np.degrees(knee_pos_arr[-1]):6.1f}°)")
                print(f"              - Range: [{np.min(knee_pos_arr):6.3f}, {np.max(knee_pos_arr):6.3f}] rad")
                print(f"              - Mean: {np.mean(knee_pos_arr):7.3f} rad, Std: {np.std(knee_pos_arr):6.3f}")
                
                print(f"Hip Velocity  - Current: {hip_vel_arr[-1]:7.3f} rad/s")
                print(f"              - Range: [{np.min(hip_vel_arr):6.3f}, {np.max(hip_vel_arr):6.3f}] rad/s")
                
                print(f"Knee Velocity - Current: {knee_vel_arr[-1]:7.3f} rad/s")
                print(f"              - Range: [{np.min(knee_vel_arr):6.3f}, {np.max(knee_vel_arr):6.3f}] rad/s")
        
        # Ankle trajectory statistics
        if self.ankle_stats['count'] > 0:
            print(f"\nANKLE TRAJECTORIES (Total messages: {self.ankle_stats['count']})")
            print("-" * 50)
            
            if len(self.ankle_stats['right_pos_x']) > 0:
                r_x_arr = np.array(self.ankle_stats['right_pos_x'])
                r_y_arr = np.array(self.ankle_stats['right_pos_y'])
                l_x_arr = np.array(self.ankle_stats['left_pos_x'])
                l_y_arr = np.array(self.ankle_stats['left_pos_y'])
                phase_arr = np.array(self.ankle_stats['time_phases'])
                
                print(f"Right Ankle   - Current: ({r_x_arr[-1]:6.3f}, {r_y_arr[-1]:6.3f}) m")
                print(f"              - X Range: [{np.min(r_x_arr):6.3f}, {np.max(r_x_arr):6.3f}] m")
                print(f"              - Y Range: [{np.min(r_y_arr):6.3f}, {np.max(r_y_arr):6.3f}] m")
                
                print(f"Left Ankle    - Current: ({l_x_arr[-1]:6.3f}, {l_y_arr[-1]:6.3f}) m")
                print(f"              - X Range: [{np.min(l_x_arr):6.3f}, {np.max(l_x_arr):6.3f}] m")
                print(f"              - Y Range: [{np.min(l_y_arr):6.3f}, {np.max(l_y_arr):6.3f}] m")
                
                print(f"Time Phase    - Current: {phase_arr[-1]:6.3f}")
                print(f"              - Range: [{np.min(phase_arr):6.3f}, {np.max(phase_arr):6.3f}]")
        
        # Publishing rate information
        current_time = rospy.Time.now()
        if hasattr(self, 'last_display_time'):
            dt = (current_time - self.last_display_time).to_sec()
            if dt > 0:
                joint_rate = (self.joint_stats['count'] - getattr(self, 'last_joint_count', 0)) / dt
                ankle_rate = (self.ankle_stats['count'] - getattr(self, 'last_ankle_count', 0)) / dt
                print(f"\nPUBLISHING RATES")
                print("-" * 20)
                print(f"Joint Trajectories:  {joint_rate:6.1f} Hz")
                print(f"Ankle Trajectories:  {ankle_rate:6.1f} Hz")
        
        self.last_display_time = current_time
        self.last_joint_count = self.joint_stats['count']
        self.last_ankle_count = self.ankle_stats['count']
        
        print("="*80)

if __name__ == '__main__':
    try:
        monitor = TrajectoryMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Trajectory Monitor shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")