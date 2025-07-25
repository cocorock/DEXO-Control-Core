#!/usr/bin/env python3

import rospy
import math
import numpy as np
import threading
import time
from exoskeleton_control.msg import GaitParams, EStopTrigger, FloorRef

class GaitPlannerNode:
    def __init__(self):
        rospy.init_node('gait_planner_node')

        # Load configuration
        self.load_configuration()

        # Gait state management
        self.is_emergency_stop = False
        self.gait_active = False
        self.current_gait_phase = 0.0  # Phase between 0-1
        self.right_gait_phase = 0.0    # Right leg phase
        self.left_gait_phase = 0.5     # Left leg phase (50% offset)
        
        # Gait timing
        self.gait_cycle_duration = 2.0  # seconds per complete gait cycle
        self.last_gait_update = rospy.Time.now()
        
        # Floor reference (default ground level)
        self.floor_plane = {
            'normal': np.array([0.0, 1.0, 0.0]),  # Upward normal
            'point': np.array([0.0, 0.0, 0.0])    # Ground level
        }
        
        # Torso state
        self.torso_orientation = 0.0  # Current torso angle (rad)
        
        # Thread safety
        self.state_lock = threading.Lock()

        # Subscribers
        rospy.Subscriber('floor_ref', FloorRef, self.floor_ref_callback)
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)
        # Note: torso_pose subscriber would be added when implemented
        
        # Publishers
        self.gait_params_pub = rospy.Publisher('gait_params', GaitParams, queue_size=1)

        # Control rate
        self.rate = rospy.Rate(self.control_frequency)

        rospy.loginfo("Gait Planner Node initialized")
        rospy.loginfo(f"Configuration: frequency={self.control_frequency}Hz, "
                     f"step_length={self.step_length:.3f}m, "
                     f"step_height={self.step_height:.3f}m")

    def load_configuration(self):
        """Load configuration parameters from ROS parameter server."""
        try:
            # Control parameters
            self.control_frequency = rospy.get_param('~control_frequency', 15.0)  # 10-20Hz
            
            # Gait parameters
            self.step_length = rospy.get_param('~gait/step_length', 0.3)  # meters
            self.step_height = rospy.get_param('~gait/step_height', 0.05)  # meters
            self.step_width = rospy.get_param('~gait/step_width', 0.2)   # meters (lateral)
            self.gait_frequency = rospy.get_param('~gait/frequency', 0.5)  # Hz (steps per second)
            
            # Closed-chain IK parameters
            self.coupling_distance = rospy.get_param('~closed_chain/coupling_distance', 0.1)  # meters (d parameter)
            self.hip_separation = rospy.get_param('~closed_chain/hip_separation', 0.3)   # meters
            self.constraint_tolerance = rospy.get_param('~closed_chain/constraint_tolerance', 0.001)  # meters
            
            # Joint limits and ranges
            self.hip_range = rospy.get_param('~joint_ranges/hip_range', math.radians(60))  # rad
            self.knee_range = rospy.get_param('~joint_ranges/knee_range', math.radians(90))  # rad
            self.torso_range = rospy.get_param('~joint_ranges/torso_range', math.radians(30))  # rad
            
            # Safety parameters
            self.max_step_deviation = rospy.get_param('~safety/max_step_deviation', 0.1)  # meters
            self.min_ground_clearance = rospy.get_param('~safety/min_ground_clearance', 0.02)  # meters
            
            # Gait planning parameters
            self.planning_horizon = rospy.get_param('~planning/horizon', 3)  # Number of steps ahead
            self.adaptive_timing = rospy.get_param('~planning/adaptive_timing', True)
            self.terrain_adaptation = rospy.get_param('~planning/terrain_adaptation', True)
            
            rospy.loginfo("Gait planner configuration loaded successfully")

        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            self.set_default_configuration()

    def set_default_configuration(self):
        """Set default configuration values."""
        self.control_frequency = 15.0
        self.step_length = 0.3
        self.step_height = 0.05
        self.step_width = 0.2
        self.gait_frequency = 0.5
        self.coupling_distance = 0.1
        self.hip_separation = 0.3
        self.constraint_tolerance = 0.001
        self.hip_range = math.radians(60)
        self.knee_range = math.radians(90)
        self.torso_range = math.radians(30)
        self.max_step_deviation = 0.1
        self.min_ground_clearance = 0.02
        self.planning_horizon = 3
        self.adaptive_timing = True
        self.terrain_adaptation = True

    def floor_ref_callback(self, msg):
        """Process floor reference plane updates."""
        try:
            # Extract floor plane information from message
            # Assuming FloorRef contains normal vector and point on plane
            with self.state_lock:
                if hasattr(msg, 'normal') and hasattr(msg, 'point'):
                    self.floor_plane['normal'] = np.array([msg.normal.x, msg.normal.y, msg.normal.z])
                    self.floor_plane['point'] = np.array([msg.point.x, msg.point.y, msg.point.z])
                    
                    rospy.loginfo(f"Floor reference updated: "
                                f"normal=({msg.normal.x:.3f}, {msg.normal.y:.3f}, {msg.normal.z:.3f})")
                    
        except Exception as e:
            rospy.logwarn(f"Error processing floor reference: {e}")

    def e_stop_callback(self, msg):
        """Handle emergency stop."""
        rospy.loginfo(f"Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            with self.state_lock:
                self.is_emergency_stop = True
                self.gait_active = False
            rospy.logwarn("Gait planning stopped due to emergency")
        else:
            with self.state_lock:
                self.is_emergency_stop = False

    def calculate_gait_phase(self):
        """Calculate current gait phase based on timing."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_gait_update).to_sec()
        self.last_gait_update = current_time
        
        if self.gait_active and not self.is_emergency_stop:
            # Update gait phase
            phase_increment = dt * self.gait_frequency
            
            with self.state_lock:
                self.current_gait_phase += phase_increment
                
                # Keep phase in [0, 1] range
                if self.current_gait_phase >= 1.0:
                    self.current_gait_phase -= 1.0
                    rospy.loginfo("Gait cycle completed")
                
                # Calculate individual leg phases
                self.right_gait_phase = self.current_gait_phase
                self.left_gait_phase = (self.current_gait_phase + 0.5) % 1.0  # 50% offset

    def calculate_foot_trajectory(self, phase, is_right_foot=True):
        """
        Calculate foot target position based on gait phase.
        
        Args:
            phase: Gait phase (0-1)
            is_right_foot: True for right foot, False for left foot
            
        Returns:
            target_position: [x, y, z] position relative to body center
        """
        try:
            # Determine stance vs swing phase
            # Phase 0-0.6: stance phase, 0.6-1.0: swing phase
            stance_duration = 0.6
            
            if phase < stance_duration:
                # Stance phase - foot on ground
                # Linear progression from back to front of step
                step_progress = phase / stance_duration
                x_offset = self.step_length * (0.5 - step_progress)  # Start ahead, move back
                y_offset = 0.0  # On ground
                
            else:
                # Swing phase - foot in air
                swing_progress = (phase - stance_duration) / (1.0 - stance_duration)
                
                # Swing trajectory (simplified arc)
                x_offset = self.step_length * (swing_progress - 0.5)  # Move from back to front
                y_offset = self.step_height * math.sin(math.pi * swing_progress)  # Arc height
            
            # Lateral offset for foot width
            z_offset = self.step_width / 2.0 if is_right_foot else -self.step_width / 2.0
            
            # Apply floor plane offset
            ground_level = self.floor_plane['point'][1]  # Y is up
            
            return np.array([x_offset, ground_level + y_offset, z_offset])
            
        except Exception as e:
            rospy.logerr(f"Error calculating foot trajectory: {e}")
            return np.array([0.0, 0.0, 0.0])

    def calculate_closed_chain_ik(self, leading_ankle_X_pos, leading_ankle_Y_pos, Hip_range, Knee_range, Torso_range):
        """
        Calculate closed-chain inverse kinematics using the specified equation:
        x_target = 2*X_max + X_min + (d*sin(theta_torso))
        
        Args:
            leading_ankle_X_pos: X position of leading ankle
            leading_ankle_Y_pos: Y position of leading ankle
            Hip_range: Allowable hip joint range (rad)
            Knee_range: Allowable knee joint range (rad)
            Torso_range: Allowable torso orientation range (rad)
            
        Returns:
            error_flag: Error status (0 = success, <0 = error)
            theta_Rhip: Right hip joint angle (rad)
            theta_Lhip: Left hip joint angle (rad)
            theta_Rknee: Right knee joint angle (rad)
            theta_Lknee: Left knee joint angle (rad)
            theta_torso: Torso orientation angle (rad)
        """
        try:
            # Get current foot positions to determine X_max and X_min
            # In a real implementation, these would come from current leg positions
            # For now, we'll use the leading ankle position and estimate the trailing position
            
            # Determine which foot is leading based on gait phase
            if self.right_gait_phase > 0.6:  # Right foot in swing (leading)
                X_max = leading_ankle_X_pos  # Leading foot position
                X_min = leading_ankle_X_pos - self.step_length  # Trailing foot position
                leading_foot_is_right = True
            else:  # Left foot in swing (leading)
                X_max = leading_ankle_X_pos  # Leading foot position  
                X_min = leading_ankle_X_pos - self.step_length  # Trailing foot position
                leading_foot_is_right = False
            
            # Apply your closed-chain inverse kinematics equation:
            # x_target = 2*X_max + X_min + (d*sin(theta_torso))
            theta_torso = self.torso_orientation
            d = self.coupling_distance  # Coupling mechanism distance parameter
            
            x_target = 2*X_max + X_min + (d * math.sin(theta_torso))
            
            rospy.loginfo(f"Closed Chain IK: X_max={X_max:.3f}, X_min={X_min:.3f}, "
                         f"theta_torso={math.degrees(theta_torso):.1f}° -> x_target={x_target:.3f}")
            
            # Calculate individual leg targets
            # Right leg target position
            if leading_foot_is_right:
                right_ankle_x = leading_ankle_X_pos
                right_ankle_y = leading_ankle_Y_pos
                # Left leg position constrained by closed-chain equation
                left_ankle_x = x_target - 2*leading_ankle_X_pos  # Derived from the equation
                left_ankle_y = leading_ankle_Y_pos  # Assume same height
            else:
                left_ankle_x = leading_ankle_X_pos
                left_ankle_y = leading_ankle_Y_pos
                # Right leg position constrained by closed-chain equation
                right_ankle_x = x_target - 2*leading_ankle_X_pos  # Derived from the equation
                right_ankle_y = leading_ankle_Y_pos  # Assume same height
            
            # Calculate hip positions (fixed relative to torso)
            hip_separation_x = self.hip_separation * math.cos(theta_torso)
            hip_separation_z = self.hip_separation * math.sin(theta_torso)
            
            # Hip positions in global frame
            right_hip_x = hip_separation_x / 2.0
            left_hip_x = -hip_separation_x / 2.0
            hip_y = 0.0  # Hip height reference
            
            # Calculate ankle positions relative to respective hips
            right_ankle_relative_x = right_ankle_x - right_hip_x
            right_ankle_relative_y = right_ankle_y - hip_y
            
            left_ankle_relative_x = left_ankle_x - left_hip_x
            left_ankle_relative_y = left_ankle_y - hip_y
            
            # Calculate joint angles for each leg using 2-link IK
            theta_Rhip, theta_Rknee = self.calculate_2link_ik(
                right_ankle_relative_x, right_ankle_relative_y
            )
            
            theta_Lhip, theta_Lknee = self.calculate_2link_ik(
                left_ankle_relative_x, left_ankle_relative_y
            )
            
            # Check joint limits
            error_flag = 0
            
            if (abs(theta_Rhip) > Hip_range or abs(theta_Lhip) > Hip_range or
                abs(theta_Rknee) > Knee_range or abs(theta_Lknee) > Knee_range or
                abs(theta_torso) > Torso_range):
                error_flag = -1
                rospy.logwarn("Joint limits exceeded in closed-chain IK")
            
            # Check for unreachable positions
            if (math.isnan(theta_Rhip) or math.isnan(theta_Lhip) or
                math.isnan(theta_Rknee) or math.isnan(theta_Lknee)):
                error_flag = -2
                rospy.logwarn("Unreachable position in closed-chain IK")
            
            # Validate the closed-chain constraint
            calculated_constraint = 2*X_max + X_min + (d * math.sin(theta_torso))
            constraint_error = abs(x_target - calculated_constraint)
            
            if constraint_error > self.constraint_tolerance:
                error_flag = -3
                rospy.logwarn(f"Closed-chain constraint violation: error={constraint_error:.6f}")
            
            rospy.loginfo(f"Closed Chain IK Result: "
                         f"R_hip={math.degrees(theta_Rhip):.1f}°, R_knee={math.degrees(theta_Rknee):.1f}°, "
                         f"L_hip={math.degrees(theta_Lhip):.1f}°, L_knee={math.degrees(theta_Lknee):.1f}°, "
                         f"torso={math.degrees(theta_torso):.1f}°")
            
            return error_flag, theta_Rhip, theta_Lhip, theta_Rknee, theta_Lknee, theta_torso
            
        except Exception as e:
            rospy.logerr(f"Error in closed chain IK calculation: {e}")
            return -4, 0.0, 0.0, 0.0, 0.0, 0.0  # Return error with zero angles

    def calculate_2link_ik(self, target_x, target_y, L1=0.44, L2=0.44):
        """
        Calculate 2-link inverse kinematics for a single leg.
        
        Args:
            target_x, target_y: Target ankle position relative to hip
            L1: Thigh length (m)
            L2: Shin length (m)
            
        Returns:
            theta1: Hip joint angle (rad)
            theta2: Knee joint angle (rad)
        """
        try:
            # Calculate distance to target
            distance = math.sqrt(target_x**2 + target_y**2)
            
            # Check reachability
            if distance > (L1 + L2) or distance < abs(L1 - L2):
                rospy.logwarn(f"Target unreachable: distance={distance:.3f}, links=({L1:.3f}, {L2:.3f})")
                return float('nan'), float('nan')
            
            # Calculate knee angle using law of cosines
            cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
            cos_theta2 = max(-1.0, min(1.0, cos_theta2))  # Clamp to valid range
            
            # Choose knee-down configuration (negative knee angle)
            theta2 = -math.acos(cos_theta2)
            
            # Calculate hip angle
            alpha = math.atan2(target_y, target_x)
            beta = math.acos((L1**2 + distance**2 - L2**2) / (2 * L1 * distance))
            theta1 = alpha - beta
            
            return theta1, theta2
            
        except Exception as e:
            rospy.logerr(f"Error in 2-link IK: {e}")
            return float('nan'), float('nan')

    def update_gait_targets(self):
        """Update gait targets based on current phase."""
        if not self.gait_active or self.is_emergency_stop:
            return None
        
        try:
            # Calculate foot positions for current phase
            right_foot_pos = self.calculate_foot_trajectory(self.right_gait_phase, is_right_foot=True)
            left_foot_pos = self.calculate_foot_trajectory(self.left_gait_phase, is_right_foot=False)
            
            # Determine leading foot (the one in swing phase or furthest forward)
            right_in_swing = self.right_gait_phase > 0.6
            left_in_swing = self.left_gait_phase > 0.6
            
            if right_in_swing and not left_in_swing:
                leading_ankle = right_foot_pos[:2]  # Right foot leading
            elif left_in_swing and not right_in_swing:
                leading_ankle = left_foot_pos[:2]   # Left foot leading
            else:
                # Both in same phase, use the one further forward
                leading_ankle = right_foot_pos[:2] if right_foot_pos[0] > left_foot_pos[0] else left_foot_pos[:2]
            
            # Calculate closed-chain IK for both legs
            error_flag, theta_Rhip, theta_Lhip, theta_Rknee, theta_Lknee, theta_torso = \
                self.calculate_closed_chain_ik(
                    leading_ankle[0],   # leading_ankle_X_pos
                    leading_ankle[1],   # leading_ankle_Y_pos
                    self.hip_range,     # Hip_range
                    self.knee_range,    # Knee_range
                    self.torso_range    # Torso_range
                )
            
            if error_flag == 0:
                # Create and return gait parameters message
                gait_msg = GaitParams()
                gait_msg.header.stamp = rospy.Time.now()
                
                # Right leg target
                gait_msg.R_target.x = right_foot_pos[0]
                gait_msg.R_target.y = right_foot_pos[1]
                
                # Left leg target  
                gait_msg.L_target.x = left_foot_pos[0]
                gait_msg.L_target.y = left_foot_pos[1]
                
                # Additional information
                gait_msg.gait_phase = self.current_gait_phase
                gait_msg.right_phase = self.right_gait_phase
                gait_msg.left_phase = self.left_gait_phase
                
                return gait_msg
            else:
                rospy.logwarn("Failed to calculate valid gait targets")
                return None
                
        except Exception as e:
            rospy.logerr(f"Error updating gait targets: {e}")
            return None

    def start_gait(self):
        """Start gait planning."""
        if not self.is_emergency_stop:
            with self.state_lock:
                self.gait_active = True
                self.current_gait_phase = 0.0
                self.right_gait_phase = 0.0
                self.left_gait_phase = 0.5
            rospy.loginfo("Gait planning started")
        else:
            rospy.logwarn("Cannot start gait - emergency stop active")

    def stop_gait(self):
        """Stop gait planning."""
        with self.state_lock:
            self.gait_active = False
        rospy.loginfo("Gait planning stopped")

    def run(self):
        """Main execution loop."""
        rospy.loginfo("Gait Planner Node running...")
        
        while not rospy.is_shutdown():
            try:
                # Update gait phase
                self.calculate_gait_phase()
                
                # Update and publish gait targets
                if self.gait_active and not self.is_emergency_stop:
                    gait_targets = self.update_gait_targets()
                    
                    if gait_targets is not None:
                        self.gait_params_pub.publish(gait_targets)
                        
                        # Log progress occasionally
                        if int(self.current_gait_phase * 100) % 10 == 0:  # Every 10% of cycle
                            rospy.loginfo(f"Gait phase: {self.current_gait_phase:.2f}, "
                                        f"R_phase: {self.right_gait_phase:.2f}, "
                                        f"L_phase: {self.left_gait_phase:.2f}")
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in gait planner loop: {e}")
                self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GaitPlannerNode()
        
        # For testing, auto-start gait after a delay
        rospy.sleep(2.0)
        node.start_gait()
        
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gait Planner Node shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error in Gait Planner Node: {e}")
