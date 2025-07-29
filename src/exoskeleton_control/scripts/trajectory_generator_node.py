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
            self.ERROR_UNREACHABLE = -333.0
            self.ERROR_JOINT_LIMITS = -444.0

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
        self.ERROR_UNREACHABLE = -333.0
        self.ERROR_JOINT_LIMITS = -444.0

    def load_trajectory_from_json(self):
        """Load trajectory data from JSON file based on the provided structure."""
        try:
            # Get package path and resolve $(find) syntax
            import rospkg
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('exoskeleton_control')
            data_path = os.path.join(package_path, 'data')
            
            # Try to find the file in multiple locations
            possible_paths = [
                self.trajectory_file,
                os.path.join(data_path, self.trajectory_file),
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
                        # Add debug info for problematic points
                        if 113 <= i <= 161:
                            rospy.logwarn(f"[DEBUG] Point {i}: ankle=({ankle_x:.3f}, {ankle_y:.3f})")
                            rospy.logwarn(f"[DEBUG] Distance: {math.sqrt(ankle_x*ankle_x + ankle_y*ankle_y):.3f}m, max_reach: {self.L1 + self.L2:.3f}m")
                            if theta_hip == self.ERROR_JOINT_LIMITS:
                                rospy.logwarn(f"[DEBUG] JOINT_LIMITS error")
                            elif theta_hip == self.ERROR_UNREACHABLE:
                                rospy.logwarn(f"[DEBUG] UNREACHABLE error")
                        rospy.logwarn(f"Skipping unreachable point {i}: ankle=({ankle_x:.3f}, {ankle_y:.3f}, Joints: hip={theta_hip}, knee={theta_knee})")
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
        Calculate forward kinematics for 2-link planar leg
        Args:
            theta_hip: Hip joint angle (rad) - zero when pointing downward
            theta_knee: Knee joint angle (rad) - zero when straight, negative for flexion
        Returns:
            x, y: Ankle position relative to hip
        """
        # Convert joint angles to geometric angles
        theta_hip_geom = theta_hip - math.pi/2  # Hip: subtract 90° offset
        theta_knee_geom = theta_knee + math.pi   # Knee: add 180° offset
        
        # Forward kinematics equations for 2-link leg
        # Hip at origin, x is forward/back, y is up/down
        x = self.L1 * math.cos(theta_hip_geom) + self.L2 * math.cos(theta_hip_geom + theta_knee_geom)
        y = self.L1 * math.sin(theta_hip_geom) + self.L2 * math.sin(theta_hip_geom + theta_knee_geom)
        
        return x, y

    def calculate_inverse_kinematics(self, x, y):
        """
        Calculate inverse kinematics for 2-link planar leg
        Args:
            x, y: Target ankle position relative to hip (m)
                  x: forward/backward (positive forward)
                  y: up/down (positive up, negative down)
        Returns:
            theta_hip, theta_knee: Joint angles (rad)
                     theta_hip: Hip angle (zero when pointing downward)
                     theta_knee: Knee angle (zero when straight, negative for flexion)
        """
        # Calculate distance from hip to ankle
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        
        if distance > max_reach:
            rospy.logwarn(f"Target unreachable: distance={distance:.3f}, max_reach={max_reach:.3f}")
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        if distance < min_reach:
            rospy.logwarn(f"Target too close: distance={distance:.3f}, min_reach={min_reach:.3f}")
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        # Handle the case when target is at origin
        if distance < 1e-6:
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        # Calculate knee angle using law of cosines
        cos_theta_knee = (self.L1*self.L1 + self.L2*self.L2 - distance*distance) / (2 * self.L1 * self.L2)
        cos_theta_knee = max(-1.0, min(1.0, cos_theta_knee))  # Clamp to valid range
        
        # Calculate knee angle - geometric angle between links
        theta_knee_geom = math.acos(cos_theta_knee)
        
        # Convert to joint angle: knee is zero when both links point downward (straight)
        # When straight: geometric angle = 180°, joint angle = 0°
        # When flexed: geometric angle < 180°, joint angle < 0° (negative flexion)
        theta_knee = theta_knee_geom - math.pi  # Subtract 180° to shift reference
        
        # Calculate hip angle using geometric approach
        gamma = math.atan2(y, x)  # Angle from hip to target point
        
        # Find the angle from hip to knee using law of cosines
        cos_phi = (self.L1*self.L1 + distance*distance - self.L2*self.L2) / (2 * self.L1 * distance)
        cos_phi = max(-1.0, min(1.0, cos_phi))  # Clamp to valid range
        phi = math.acos(cos_phi)
        
        # Hip angle (for knee-down configuration)
        theta_hip_raw = gamma - phi
        
        # Adjust for hip joint zero reference (zero when pointing downward)
        theta_hip = theta_hip_raw + math.pi/2  # Add 90° to shift reference
        
        # Normalize angles to [-pi, pi]
        theta_hip = self.normalize_angle(theta_hip)
        theta_knee = self.normalize_angle(theta_knee)
        
        # Check joint limits with small tolerance for numerical precision
        hip_tolerance = 0.1 * math.pi/180  # 0.1 degree tolerance
        hip_in_limits = (theta_hip >= (self.theta1_min - hip_tolerance) and theta_hip <= self.theta1_max)
        knee_in_limits = (theta_knee >= self.theta2_min and theta_knee <= self.theta2_max)
        
        if not hip_in_limits or not knee_in_limits:
            # Try alternative configuration (knee-up) if first fails
            theta_knee_alt_geom = -math.acos(cos_theta_knee)  # Alternative geometric angle
            theta_knee_alt = theta_knee_alt_geom - math.pi  # Convert to joint angle
            theta_hip_alt_raw = gamma + phi
            
            # Apply same reference shift for alternative configuration
            theta_hip_alt = theta_hip_alt_raw + math.pi/2  # Add 90° to shift reference
            
            theta_hip_alt = self.normalize_angle(theta_hip_alt)
            theta_knee_alt = self.normalize_angle(theta_knee_alt)
            
            # Check if alternative configuration is within limits
            hip_alt_in_limits = (theta_hip_alt >= self.theta1_min and theta_hip_alt <= self.theta1_max)
            knee_alt_in_limits = (theta_knee_alt >= self.theta2_min and theta_knee_alt <= self.theta2_max)
            
            if hip_alt_in_limits and knee_alt_in_limits:
                rospy.loginfo_throttle(10.0, f"Using alternative IK configuration: hip={math.degrees(theta_hip_alt):.1f}° knee={math.degrees(theta_knee_alt):.1f}°")
                return theta_hip_alt, theta_knee_alt
            
            # Both configurations exceed limits
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
        trajectory_msg.Rhip_pos_ref = traj_point['right_leg_positions'][0]  # Hip position
        trajectory_msg.Rknee_pos_ref = traj_point['right_leg_positions'][1]  # Knee position
        trajectory_msg.Rhip_vel_ref = traj_point['right_leg_velocities'][0]  # Hip velocity
        trajectory_msg.Rknee_vel_ref = traj_point['right_leg_velocities'][1]  # Knee velocity
        
        # Left leg data (placeholder - same as right leg for now)
        trajectory_msg.Lhip_pos_ref = traj_point['left_leg_positions'][0]  # Hip position
        trajectory_msg.Lknee_pos_ref = traj_point['left_leg_positions'][1]  # Knee position
        trajectory_msg.Lhip_vel_ref = traj_point['left_leg_velocities'][0]  # Hip velocity
        trajectory_msg.Lknee_vel_ref = traj_point['left_leg_velocities'][1]  # Knee velocity
        
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
