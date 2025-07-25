#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
import os
from exoskeleton_control.msg import GaitParams, JointsTrajectory, EStopTrigger

class TrajectoryGeneratorNode:
    def __init__(self):
        rospy.init_node('trajectory_generator_node')

        # Load configuration parameters
        self.load_configuration()

        # Trajectory state
        self.is_emergency_stop = False
        self.trajectory_active = False
        self.current_trajectory_index = 0
        self.trajectory_data = None
        self.trajectory_length = 0

        # Subscribers
        rospy.Subscriber('gait_params', GaitParams, self.gait_params_callback)
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)

        # Publishers
        self.joints_trajectory_pub = rospy.Publisher('joints_trajectory', JointsTrajectory, queue_size=10)

        # Load trajectory from JSON for testing
        self.load_trajectory_from_json()

        # Control rate
        self.rate = rospy.Rate(self.control_frequency)

        rospy.loginfo("Trajectory Generator Node initialized")
        if self.trajectory_data:
            rospy.loginfo(f"Loaded trajectory with {self.trajectory_length} data points")
        else:
            rospy.logwarn("No trajectory data loaded - waiting for gait_params")

    def load_configuration(self):
        """Load configuration parameters from ROS parameter server."""
        try:
            # Control parameters
            self.control_frequency = rospy.get_param('~control_frequency', 15)  # 15-20Hz
            
            # Arm/leg parameters
            self.L1 = rospy.get_param('~leg_parameters/L1', 0.44)  # Thigh length (m)
            self.L2 = rospy.get_param('~leg_parameters/L2', 0.44)  # Shin length (m)

            # Joint limits (in radians)
            self.theta1_min = math.radians(rospy.get_param('~joint_limits/hip_min_deg', -30))
            self.theta1_max = math.radians(rospy.get_param('~joint_limits/hip_max_deg', 90))
            self.theta2_min = math.radians(rospy.get_param('~joint_limits/knee_min_deg', -100))
            self.theta2_max = math.radians(rospy.get_param('~joint_limits/knee_max_deg', 0))

            # Trajectory file path
            self.trajectory_file = rospy.get_param('~trajectory_file', 'new_processed_gait_data#39_1_json_structure_1.json')
            
            # Trajectory parameters
            self.loop_trajectory = rospy.get_param('~loop_trajectory', True)
            self.trajectory_scale = rospy.get_param('~trajectory_scale', 1.0)  # Scale factor for positions
            
            # Error codes
            self.ERROR_UNREACHABLE = -5.0
            self.ERROR_JOINT_LIMITS = -6.0

            rospy.loginfo("Trajectory generator configuration loaded successfully")

        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            self.set_default_configuration()

    def set_default_configuration(self):
        """Set default configuration values."""
        self.control_frequency = 15
        self.L1 = 0.44  # Thigh length
        self.L2 = 0.44  # Shin length
        self.theta1_min = math.radians(-30)
        self.theta1_max = math.radians(90)
        self.theta2_min = math.radians(-100)
        self.theta2_max = math.radians(0)
        self.trajectory_file = 'trajectory.json'
        self.loop_trajectory = True
        self.trajectory_scale = 1.0
        self.ERROR_UNREACHABLE = -5.0
        self.ERROR_JOINT_LIMITS = -6.0

    def load_trajectory_from_json(self):
        """Load trajectory data from JSON file based on the provided structure."""
        try:
            # Try to find the file in multiple locations
            possible_paths = [
                self.trajectory_file,
                os.path.join(rospy.get_param('~package_path', '.'), self.trajectory_file),
                os.path.join('/tmp', self.trajectory_file),
                os.path.expanduser(f'~/{self.trajectory_file}')
            ]
            
            file_found = False
            for path in possible_paths:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        raw_data = json.load(f)
                    file_found = True
                    rospy.loginfo(f"Trajectory file loaded from: {path}")
                    break
            
            if not file_found:
                rospy.logerr(f"Trajectory file not found in any of these locations: {possible_paths}")
                return False

            # Parse the JSON structure based on the provided format
            if not raw_data or len(raw_data) == 0:
                rospy.logerr("Empty trajectory data")
                return False

            # Extract the first demonstration
            demo_data = raw_data[0]
            
            # Validate required fields
            required_fields = ['time', 'ankle_pos_FR1', 'ankle_pos_FR1_velocity']
            for field in required_fields:
                if field not in demo_data:
                    rospy.logerr(f"Missing required field: {field}")
                    return False

            # Extract trajectory data
            time_data = demo_data['time']
            ankle_pos_data = demo_data['ankle_pos_FR1']  # Right leg ankle positions
            ankle_vel_data = demo_data['ankle_pos_FR1_velocity']  # Right leg ankle velocities
            
            self.trajectory_length = len(time_data)
            
            # Process trajectory data
            self.trajectory_data = {
                'time': [],
                'positions': [],  # [hip_pos, knee_pos] for each timestep
                'velocities': []  # [hip_vel, knee_vel] for each timestep
            }
            
            rospy.loginfo(f"Processing {self.trajectory_length} trajectory points...")
            
            successful_points = 0
            for i in range(self.trajectory_length):
                try:
                    # Extract time (nested array structure: [[time]])
                    if isinstance(time_data[i], list) and len(time_data[i]) > 0:
                        time_val = time_data[i][0]
                    else:
                        time_val = float(i) * 0.01  # Default 100Hz if time format is wrong
                    
                    # Extract ankle position (nested array structure: [[x, y]])
                    if (isinstance(ankle_pos_data[i], list) and len(ankle_pos_data[i]) >= 2):
                        ankle_x = ankle_pos_data[i][0] * self.trajectory_scale
                        ankle_y = ankle_pos_data[i][1] * self.trajectory_scale
                    else:
                        rospy.logwarn(f"Invalid ankle position data at index {i}")
                        continue
                    
                    # Extract ankle velocity (nested array structure: [[vx, vy]])
                    if (isinstance(ankle_vel_data[i], list) and len(ankle_vel_data[i]) >= 2):
                        ankle_vx = ankle_vel_data[i][0] * self.trajectory_scale
                        ankle_vy = ankle_vel_data[i][1] * self.trajectory_scale
                    else:
                        rospy.logwarn(f"Invalid ankle velocity data at index {i}")
                        continue

                    # Convert ankle position to joint angles using inverse kinematics
                    theta_hip, theta_knee = self.calculate_inverse_kinematics(ankle_x, ankle_y)
                    
                    # Skip points that result in errors
                    if theta_hip < -4.0 or theta_knee < -4.0:  # Error codes
                        rospy.logwarn(f"Skipping unreachable point {i}: ankle=({ankle_x:.3f}, {ankle_y:.3f})")
                        continue
                    
                    # Calculate joint velocities using Jacobian
                    joint_vx, joint_vy = self.calculate_joint_velocities(
                        theta_hip, theta_knee, ankle_vx, ankle_vy
                    )
                    
                    # Store processed data
                    self.trajectory_data['time'].append(time_val)
                    self.trajectory_data['positions'].append([theta_hip, theta_knee])
                    self.trajectory_data['velocities'].append([joint_vx, joint_vy])
                    
                    successful_points += 1
                    
                except Exception as e:
                    rospy.logwarn(f"Error processing trajectory point {i}: {e}")
                    continue
            
            if successful_points == 0:
                rospy.logerr("No valid trajectory points could be processed")
                return False
            
            self.trajectory_length = successful_points
            rospy.loginfo(f"Successfully processed {successful_points}/{len(time_data)} trajectory points")
            
            # Log some statistics
            if self.trajectory_data['positions']:
                positions = np.array(self.trajectory_data['positions'])
                rospy.loginfo(f"Hip angle range: {math.degrees(np.min(positions[:, 0])):.1f}° to {math.degrees(np.max(positions[:, 0])):.1f}°")
                rospy.loginfo(f"Knee angle range: {math.degrees(np.min(positions[:, 1])):.1f}° to {math.degrees(np.max(positions[:, 1])):.1f}°")
            
            return True

        except Exception as e:
            rospy.logerr(f"Error loading trajectory from JSON: {e}")
            return False

    def calculate_forward_kinematics(self, theta_hip, theta_knee):
        """
        Calculate forward kinematics for 2-link planar arm
        Args:
            theta_hip: Hip joint angle (rad)
            theta_knee: Knee joint angle (rad)
        Returns:
            x, y: Ankle position relative to hip
        """
        # Forward kinematics equations for 2-link arm
        # Assuming hip is at origin, positive x is forward, positive y is up
        x = self.L1 * math.cos(theta_hip) + self.L2 * math.cos(theta_hip + theta_knee)
        y = self.L1 * math.sin(theta_hip) + self.L2 * math.sin(theta_hip + theta_knee)
        
        return x, y

    def calculate_inverse_kinematics(self, x, y):
        """
        Calculate inverse kinematics for 2-link planar arm (leg)
        Args:
            x, y: Target ankle position relative to hip
        Returns:
            theta_hip, theta_knee: Joint angles (rad)
        """
        # Calculate distance from hip to ankle
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if distance > (self.L1 + self.L2):
            rospy.logwarn(f"Target unreachable: distance={distance:.3f}, max_reach={self.L1 + self.L2:.3f}")
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        if distance < abs(self.L1 - self.L2):
            rospy.logwarn(f"Target too close: distance={distance:.3f}, min_reach={abs(self.L1 - self.L2):.3f}")
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        # Calculate knee angle using law of cosines
        cos_theta2 = (distance*distance - self.L1*self.L1 - self.L2*self.L2) / (2 * self.L1 * self.L2)
        
        # Clamp cos_theta2 to valid range to handle numerical errors
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # Choose elbow-down configuration (negative knee angle for leg)
        theta_knee = -math.acos(cos_theta2)
        
        # Calculate hip angle
        alpha = math.atan2(y, x)  # Angle from hip to target
        beta = math.acos((self.L1*self.L1 + distance*distance - self.L2*self.L2) / (2 * self.L1 * distance))
        
        theta_hip = alpha - beta
        
        # Normalize angles to [-pi, pi]
        theta_hip = self.normalize_angle(theta_hip)
        theta_knee = self.normalize_angle(theta_knee)
        
        # Check joint limits
        if (theta_hip < self.theta1_min or theta_hip > self.theta1_max or 
            theta_knee < self.theta2_min or theta_knee > self.theta2_max):
            rospy.logwarn_throttle(5.0, f"Joint limits exceeded: hip={math.degrees(theta_hip):.1f}° knee={math.degrees(theta_knee):.1f}°")
            return self.ERROR_JOINT_LIMITS, self.ERROR_JOINT_LIMITS
        
        return theta_hip, theta_knee

    def calculate_joint_velocities(self, theta_hip, theta_knee, ankle_vx, ankle_vy):
        """
        Calculate joint velocities from ankle velocities using Jacobian
        Args:
            theta_hip, theta_knee: Current joint angles (rad)
            ankle_vx, ankle_vy: Desired ankle velocities (m/s)
        Returns:
            joint_vel_hip, joint_vel_knee: Joint velocities (rad/s)
        """
        try:
            # Calculate Jacobian matrix
            # J = [[-L1*sin(th1) - L2*sin(th1+th2), -L2*sin(th1+th2)],
            #      [ L1*cos(th1) + L2*cos(th1+th2),  L2*cos(th1+th2)]]
            
            s1 = math.sin(theta_hip)
            c1 = math.cos(theta_hip)
            s12 = math.sin(theta_hip + theta_knee)
            c12 = math.cos(theta_hip + theta_knee)
            
            J = np.array([
                [-self.L1 * s1 - self.L2 * s12, -self.L2 * s12],
                [ self.L1 * c1 + self.L2 * c12,  self.L2 * c12]
            ])
            
            # Check for singularities
            det_J = np.linalg.det(J)
            if abs(det_J) < 1e-6:
                rospy.logwarn_throttle(1.0, "Near singular configuration, using zero velocities")
                return 0.0, 0.0
            
            # Calculate joint velocities: q_dot = J^(-1) * x_dot
            ankle_vel = np.array([ankle_vx, ankle_vy])
            joint_vel = np.linalg.solve(J, ankle_vel)
            
            return joint_vel[0], joint_vel[1]
            
        except Exception as e:
            rospy.logwarn(f"Error calculating joint velocities: {e}")
            return 0.0, 0.0

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def gait_params_callback(self, msg):
        """Process gait parameters - future implementation for GMR."""
        rospy.loginfo("Received gait_params - GMR implementation pending")
        # TODO: Implement GMR-based trajectory generation
        pass

    def e_stop_callback(self, msg):
        """Handle emergency stop."""
        rospy.loginfo(f"Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            self.is_emergency_stop = True
            self.trajectory_active = False
        else:
            self.is_emergency_stop = False

    def get_current_trajectory_point(self):
        """Get current trajectory point and advance index."""
        if not self.trajectory_data or self.trajectory_length == 0:
            return None
        
        # Get current point
        current_pos = self.trajectory_data['positions'][self.current_trajectory_index]
        current_vel = self.trajectory_data['velocities'][self.current_trajectory_index]
        current_time = self.trajectory_data['time'][self.current_trajectory_index]
        
        # Advance index
        self.current_trajectory_index += 1
        
        # Handle looping
        if self.current_trajectory_index >= self.trajectory_length:
            if self.loop_trajectory:
                self.current_trajectory_index = 0
                rospy.loginfo("Trajectory loop completed, restarting")
            else:
                rospy.loginfo("Trajectory completed")
                self.trajectory_active = False
        
        return {
            'time': current_time,
            'right_leg_positions': current_pos,
            'right_leg_velocities': current_vel,
            'left_leg_positions': [0.0, 0.0],  # TODO: Add left leg data
            'left_leg_velocities': [0.0, 0.0]  # TODO: Add left leg data
        }

    def publish_trajectory(self):
        """Publish joint trajectory message."""
        if self.is_emergency_stop:
            return
        
        # Get current trajectory point
        traj_point = self.get_current_trajectory_point()
        if not traj_point:
            return
        
        # Create and populate trajectory message
        trajectory_msg = JointsTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        
        # Right leg data (from JSON)
        trajectory_msg.right_leg.positions = traj_point['right_leg_positions']
        trajectory_msg.right_leg.velocities = traj_point['right_leg_velocities']
        
        # Left leg data (placeholder - same as right leg for now)
        trajectory_msg.left_leg.positions = traj_point['left_leg_positions']
        trajectory_msg.left_leg.velocities = traj_point['left_leg_velocities']
        
        # Publish trajectory
        self.joints_trajectory_pub.publish(trajectory_msg)
        
        # Log progress occasionally
        if self.current_trajectory_index % 50 == 0:
            hip_deg = math.degrees(traj_point['right_leg_positions'][0])
            knee_deg = math.degrees(traj_point['right_leg_positions'][1])
            rospy.loginfo(f"Trajectory point {self.current_trajectory_index}/{self.trajectory_length}: "
                         f"hip={hip_deg:.1f}°, knee={knee_deg:.1f}°")

    def start_trajectory(self):
        """Start trajectory playback."""
        if not self.trajectory_data:
            rospy.logwarn("No trajectory data available")
            return False
        
        if self.is_emergency_stop:
            rospy.logwarn("Cannot start trajectory - emergency stop active")
            return False
        
        self.trajectory_active = True
        self.current_trajectory_index = 0
        rospy.loginfo("Trajectory playback started")
        return True

    def stop_trajectory(self):
        """Stop trajectory playback."""
        self.trajectory_active = False
        rospy.loginfo("Trajectory playback stopped")

    def run(self):
        """Main execution loop."""
        rospy.loginfo("Trajectory Generator Node running...")
        
        # Auto-start trajectory if data is available
        if self.trajectory_data and not self.trajectory_active:
            self.start_trajectory()
        
        while not rospy.is_shutdown():
            try:
                if self.trajectory_active and not self.is_emergency_stop:
                    self.publish_trajectory()
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in trajectory generator loop: {e}")
                self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TrajectoryGeneratorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Trajectory Generator Node shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error in Trajectory Generator Node: {e}")
