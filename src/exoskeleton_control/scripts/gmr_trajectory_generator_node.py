#!/usr/bin/env python3

import rospy
import numpy as np
import math
import joblib
import os
from sklearn.mixture import GaussianMixture
from exoskeleton_control.msg import GaitParams, JointsTrajectory, EStopTrigger, Trigger, FSMState

class GMRTrajectoryGeneratorNode:
    def __init__(self):
        rospy.init_node('gmr_trajectory_generator_node')

        # Load configuration parameters
        self.load_configuration()

        # Trajectory state
        self.is_emergency_stop = False
        self.trajectory_active = False
        self.current_time = 0.0
        self.trajectory_duration = 1.0  # Duration of one gait cycle
        
        # GMM model for trajectory generation
        self.gmm_model = None
        self.model_data = None
        
        # System state tracking (synchronized with emergency stop node)
        self.system_state = "INIT"
        self.previous_state = "INIT"

        # Subscribers
        rospy.Subscriber('gait_params', GaitParams, self.gait_params_callback)
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)
        rospy.Subscriber('fsm_state', FSMState, self.fsm_state_callback)

        # Publishers
        self.joints_trajectory_pub = rospy.Publisher('joints_trajectory', JointsTrajectory, queue_size=10)
        self.cycle_finished_pub = rospy.Publisher('cycle_finished', Trigger, queue_size=1)
        self.e_stop_trigger_pub = rospy.Publisher('e_stop_trigger', EStopTrigger, queue_size=1)

        # Load GMM model
        self.load_gmm_model()

        # Control rate
        self.rate = rospy.Rate(self.control_frequency)

        rospy.loginfo("gmr: GMR Trajectory Generator Node initialized")
        if self.gmm_model:
            rospy.loginfo(f"gmr: Loaded GMM model with {self.model_data['n_components']} components")
        else:
            rospy.logwarn("gmr: No GMM model loaded - waiting for model file")

    def load_configuration(self):
        """Load configuration parameters from ROS parameter server."""
        try:
            # Control parameters
            self.control_frequency = rospy.get_param('~control_frequency', 100)
            
            # Arm/leg parameters
            self.L1 = rospy.get_param('~leg_parameters/L1', 0.425)  # Thigh length (m)
            self.L2 = rospy.get_param('~leg_parameters/L2', 0.45)   # Shin length (m)

            # Joint limits (in radians)
            self.theta1_min = math.radians(rospy.get_param('~joint_limits/hip_min_deg', -10))
            self.theta1_max = math.radians(rospy.get_param('~joint_limits/hip_max_deg', 50))
            self.theta2_min = math.radians(rospy.get_param('~joint_limits/knee_min_deg', -60))
            self.theta2_max = math.radians(rospy.get_param('~joint_limits/knee_max_deg', 0))

            # GMM model file path
            self.model_file = rospy.get_param('~model_file', 'src/exoskeleton_control/data/gmm_gait_model.pkl')
            
            # Trajectory parameters
            self.trajectory_scale = rospy.get_param('~trajectory_scale', 1.0)
            self.loop_trajectory = rospy.get_param('~loop_trajectory', True)
            
            # Error codes
            self.ERROR_UNREACHABLE = -0.123
            self.ERROR_JOINT_LIMITS = -0.321

            rospy.loginfo("gmr: GMR trajectory generator configuration loaded successfully")

        except Exception as e:
            rospy.logerr(f"gmr: Error loading configuration: {e}")
            self.set_default_configuration()

    def set_default_configuration(self):
        """Set default configuration values."""
        self.control_frequency = 100
        self.L1 = 0.425
        self.L2 = 0.45
        self.theta1_min = math.radians(-10)
        self.theta1_max = math.radians(50)
        self.theta2_min = math.radians(-60)
        self.theta2_max = math.radians(0)
        self.model_file = 'src/exoskeleton_control/data/gmm_gait_model.pkl'
        self.trajectory_scale = 1.0
        self.loop_trajectory = True
        self.ERROR_UNREACHABLE = -0.123
        self.ERROR_JOINT_LIMITS = -0.321

    def load_gmm_model(self):
        """Load the trained GMM model from file."""
        try:
            # Try to find the model file in multiple locations
            possible_paths = [
                self.model_file,
                os.path.expanduser(f'~/{self.model_file}'),
                os.path.join('/tmp', os.path.basename(self.model_file))
            ]
            
            # Try with ROS package path
            try:
                import rospkg
                rospack = rospkg.RosPack()
                package_path = rospack.get_path('exoskeleton_control')
                possible_paths.insert(1, os.path.join(package_path, 'data', 'gmm_gait_model.pkl'))
            except:
                pass
            
            file_found = False
            for path in possible_paths:
                if os.path.exists(path):
                    self.model_data = joblib.load(path)
                    self.gmm_model = self.model_data['gmm_model']
                    file_found = True
                    rospy.loginfo(f"gmr: GMM model loaded from: {path}")
                    break
            
            if not file_found:
                rospy.logerr(f"gmr: GMM model file not found in any of these locations: {possible_paths}")
                return False

            # Validate model structure
            required_fields = ['gmm_model', 'data_structure', 'n_components']
            for field in required_fields:
                if field not in self.model_data:
                    rospy.logerr(f"gmr: Missing required field in model: {field}")
                    return False

            rospy.loginfo(f"gmr: Model validation successful")
            rospy.loginfo(f"gmr: - Components: {self.model_data['n_components']}")
            rospy.loginfo(f"gmr: - Data dimension: {self.model_data['data_structure']['total_dim']}")
            
            return True

        except Exception as e:
            rospy.logerr(f"gmr: Error loading GMM model: {e}")
            return False

    def gaussian_mixture_regression(self, input_time):
        """
        Perform Gaussian Mixture Regression (GMR) to generate trajectory point at given time.
        
        Args:
            input_time: Time value (0.0 to 1.0 for normalized gait cycle)
            
        Returns:
            Dictionary with position and velocity data, or None if error
        """
        if self.gmm_model is None:
            rospy.logwarn("gmr: No GMM model available for regression")
            return None

        try:
            # Normalize input time to [0, 1] range
            normalized_time = max(0.0, min(1.0, input_time))
            
            # Input dimension (time only)
            input_dim = [0]  # Time dimension index
            output_dims = [1, 2, 3, 4]  # Position and velocity dimensions (pos_x, pos_y, vel_x, vel_y)
            
            # Query point
            query_point = np.array([[normalized_time]])
            
            # Get component responsibilities
            log_prob = self.gmm_model.score_samples(np.column_stack([
                query_point, 
                np.zeros((1, len(output_dims)))  # Dummy values for output dimensions
            ]))
            
            # Alternative approach: Compute responsibilities for each component
            responsibilities = []
            for k in range(self.gmm_model.n_components):
                mean_input = self.gmm_model.means_[k, input_dim]
                cov_input = self.gmm_model.covariances_[k][np.ix_(input_dim, input_dim)]
                
                # Compute Gaussian probability for input
                diff = query_point - mean_input
                exp_term = -0.5 * np.dot(np.dot(diff, np.linalg.inv(cov_input)), diff.T)
                prob = self.gmm_model.weights_[k] * np.exp(exp_term) / np.sqrt(2 * np.pi * np.linalg.det(cov_input))
                responsibilities.append(prob[0, 0])
            
            # Normalize responsibilities
            total_resp = sum(responsibilities)
            if total_resp > 0:
                responsibilities = [r / total_resp for r in responsibilities]
            else:
                rospy.logwarn("gmr: Zero total responsibility - using uniform weights")
                responsibilities = [1.0 / self.gmm_model.n_components] * self.gmm_model.n_components
            
            # Weighted regression
            predicted_output = np.zeros(len(output_dims))
            
            for k in range(self.gmm_model.n_components):
                if responsibilities[k] < 1e-6:
                    continue
                
                # Extract means and covariances for this component
                mean_input = self.gmm_model.means_[k, input_dim]
                mean_output = self.gmm_model.means_[k, output_dims]
                
                cov_input = self.gmm_model.covariances_[k][np.ix_(input_dim, input_dim)]
                cov_output = self.gmm_model.covariances_[k][np.ix_(output_dims, output_dims)]
                cov_cross = self.gmm_model.covariances_[k][np.ix_(output_dims, input_dim)]
                
                # GMR formula: μ_y + Σ_yx * Σ_xx^-1 * (x - μ_x)
                diff_input = query_point - mean_input
                conditional_mean = mean_output + np.dot(np.dot(cov_cross, np.linalg.inv(cov_input)), diff_input.T).flatten()
                
                # Weight by responsibility
                predicted_output += responsibilities[k] * conditional_mean
            
            # Apply trajectory scaling
            predicted_output[:2] *= self.trajectory_scale  # Scale positions
            predicted_output[2:] *= self.trajectory_scale * 0.25  # Scale velocities (with speed factor)
            
            return {
                'ankle_pos': predicted_output[:2],     # [x, y] position
                'ankle_vel': predicted_output[2:4],    # [vx, vy] velocity
                'time': normalized_time
            }

        except Exception as e:
            rospy.logerr(f"gmr: Error in GMR: {e}")
            return None

    def calculate_inverse_kinematics(self, x, y):
        """
        Calculate inverse kinematics for 2-link planar leg
        Args:
            x, y: Target ankle position relative to hip (m)
        Returns:
            theta_hip, theta_knee: Joint angles (rad)
        """
        # Calculate distance from hip to ankle
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        
        if distance > max_reach:
            rospy.logwarn_throttle(1.0, f"gmr: Target unreachable: distance={distance:.3f}, max_reach={max_reach:.3f}")
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        if distance < min_reach:
            rospy.logwarn_throttle(1.0, f"gmr: Target too close: distance={distance:.3f}, min_reach={min_reach:.3f}")
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
        
        # Check joint limits
        hip_in_limits = (theta_hip >= self.theta1_min and theta_hip <= self.theta1_max)
        knee_in_limits = (theta_knee >= self.theta2_min and theta_knee <= self.theta2_max)
        
        if not hip_in_limits or not knee_in_limits:
            # Try alternative configuration
            theta_knee_alt_geom = -math.acos(cos_theta_knee)
            theta_knee_alt = theta_knee_alt_geom - math.pi
            theta_hip_alt_raw = gamma + phi
            theta_hip_alt = theta_hip_alt_raw + math.pi/2
            
            theta_hip_alt = self.normalize_angle(theta_hip_alt)
            theta_knee_alt = self.normalize_angle(theta_knee_alt)
            
            # Check if alternative configuration is within limits
            hip_alt_in_limits = (theta_hip_alt >= self.theta1_min and theta_hip_alt <= self.theta1_max)
            knee_alt_in_limits = (theta_knee_alt >= self.theta2_min and theta_knee_alt <= self.theta2_max)
            
            if hip_alt_in_limits and knee_alt_in_limits:
                return theta_hip_alt, theta_knee_alt
            
            # Both configurations exceed limits
            rospy.logwarn_throttle(5.0, f"gmr: Joint limits exceeded for target ({x:.3f}, {y:.3f})")
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
                rospy.logwarn_throttle(1.0, "gmr: Near singular configuration, using zero velocities")
                return 0.0, 0.0
            
            # Calculate joint velocities: q_dot = J^(-1) * x_dot
            ankle_vel = np.array([ankle_vx, ankle_vy])
            joint_vel = np.linalg.solve(J, ankle_vel)
            
            return joint_vel[0], joint_vel[1]
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"gmr: Error calculating joint velocities: {e}")
            return 0.0, 0.0

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def generate_trajectory_point(self, time_phase):
        """Generate a single trajectory point using GMR."""
        if self.gmm_model is None:
            return None
        
        # Use GMR to get ankle position and velocity
        gmr_result = self.gaussian_mixture_regression(time_phase)
        if gmr_result is None:
            return None
        
        # Convert ankle position to joint angles
        ankle_x, ankle_y = gmr_result['ankle_pos']
        ankle_vx, ankle_vy = gmr_result['ankle_vel']
        
        # Calculate inverse kinematics
        theta_hip, theta_knee = self.calculate_inverse_kinematics(ankle_x, ankle_y)
        
        # Skip if IK failed
        if theta_hip < -4.0 or theta_knee < -4.0:
            rospy.logwarn_throttle(1.0, f"gmr: IK failed for ankle position ({ankle_x:.3f}, {ankle_y:.3f})")
            return None
        
        # Calculate joint velocities
        joint_vel_hip, joint_vel_knee = self.calculate_joint_velocities(
            theta_hip, theta_knee, ankle_vx, ankle_vy
        )
        
        return {
            'positions': [theta_hip, theta_knee],
            'velocities': [joint_vel_hip, joint_vel_knee],
            'ankle_pos': [ankle_x, ankle_y],
            'ankle_vel': [ankle_vx, ankle_vy],
            'time': gmr_result['time']
        }

    def gait_params_callback(self, msg):
        """Process gait parameters - could be used for future parameter adjustment."""
        rospy.loginfo("gmr: Received gait_params")
        # TODO: Use gait parameters to modify trajectory generation
        pass

    def e_stop_callback(self, msg):
        """Handle emergency stop."""
        rospy.loginfo(f"gmr: Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            self.is_emergency_stop = True
            self.trajectory_active = False
        else:
            self.is_emergency_stop = False

    def fsm_state_callback(self, msg):
        """Handle FSM state updates from emergency stop node."""
        rospy.loginfo_throttle(10.0, f"gmr: Received FSM state: {msg.state}")
        self.handle_state_transition(msg.state)

    def handle_state_transition(self, new_state):
        """Handle state transitions and perform appropriate actions."""
        if self.previous_state == new_state:
            return  # No state change
            
        rospy.loginfo(f"gmr: State transition: {self.previous_state} -> {new_state}")
        
        # Handle transitions based on current state machine logic
        if new_state == "WALKING" and self.previous_state == "READY":
            # Start trajectory when entering WALKING from READY
            if not self.trajectory_active:
                self.start_trajectory()
                rospy.loginfo("gmr: Started GMR trajectory for WALKING state")
        elif new_state == "STOPPING" and self.previous_state == "WALKING":
            # Continue trajectory but prepare to signal cycle finished
            rospy.loginfo("gmr: Entering STOPPING state - will signal cycle finished when trajectory completes")
        elif new_state == "READY":
            # Stop trajectory when entering READY
            if self.trajectory_active:
                self.stop_trajectory()
                rospy.loginfo("gmr: Stopped GMR trajectory for READY state")
            
        self.previous_state = self.system_state
        self.system_state = new_state

    def start_trajectory(self):
        """Start trajectory playback."""
        if self.gmm_model is None:
            rospy.logwarn("gmr: No GMM model available - cannot start trajectory")
            return False
        
        if self.is_emergency_stop:
            rospy.logwarn("gmr: Cannot start trajectory - emergency stop active")
            return False
        
        self.trajectory_active = True
        self.current_time = 0.0
        rospy.loginfo("gmr: GMR trajectory playback started")
        return True

    def stop_trajectory(self):
        """Stop trajectory playback."""
        self.trajectory_active = False
        rospy.loginfo("gmr: GMR trajectory playback stopped")

    def send_cycle_finished(self):
        """Send cycle finished signal to emergency stop node."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.cycle_finished_pub.publish(msg)
        rospy.loginfo("gmr: Cycle finished signal sent")

    def publish_trajectory(self):
        """Publish joint trajectory message based on system state."""
        if self.is_emergency_stop:
            return
        
        # Create trajectory message
        trajectory_msg = JointsTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        
        # Default safe values
        safe_pos = [0.0, 0.0]
        safe_vel = [0.0, 0.0]
        
        # Set values based on system state
        if self.system_state == "READY":
            # READY state: publish safe position with zero velocities
            trajectory_msg.Rhip_pos_ref = safe_pos[0]
            trajectory_msg.Rknee_pos_ref = safe_pos[1]
            trajectory_msg.Rhip_vel_ref = 0.0
            trajectory_msg.Rknee_vel_ref = 0.0
            trajectory_msg.Lhip_pos_ref = safe_pos[0]
            trajectory_msg.Lknee_pos_ref = safe_pos[1]
            trajectory_msg.Lhip_vel_ref = 0.0
            trajectory_msg.Lknee_vel_ref = 0.0
            
        elif self.system_state == "WALKING" or self.system_state == "STOPPING":
            # WALKING/STOPPING states: use GMR to generate trajectory
            if self.trajectory_active and self.gmm_model:
                # Calculate current phase in gait cycle (0.0 to 1.0)
                time_phase = (self.current_time / self.trajectory_duration) % 1.0
                
                # Generate trajectory point using GMR
                traj_point = self.generate_trajectory_point(time_phase)
                
                if traj_point is not None:
                    # Use generated trajectory
                    pos = traj_point['positions']
                    vel = traj_point['velocities']
                    
                    trajectory_msg.Rhip_pos_ref = pos[0]
                    trajectory_msg.Rknee_pos_ref = pos[1]
                    trajectory_msg.Rhip_vel_ref = vel[0]
                    trajectory_msg.Rknee_vel_ref = vel[1]
                    
                    # Mirror for left leg (or use different model if available)
                    trajectory_msg.Lhip_pos_ref = pos[0]
                    trajectory_msg.Lknee_pos_ref = pos[1]
                    trajectory_msg.Lhip_vel_ref = vel[0]
                    trajectory_msg.Lknee_vel_ref = vel[1]
                    
                    # Log progress occasionally
                    if int(self.current_time * self.control_frequency) % 50 == 0:
                        hip_deg = math.degrees(pos[0])
                        knee_deg = math.degrees(pos[1])
                        rospy.loginfo(f"gmr: GMR trajectory t={time_phase:.2f}: hip={hip_deg:.1f}°, knee={knee_deg:.1f}°")
                else:
                    # Fallback to safe values if GMR fails
                    trajectory_msg.Rhip_pos_ref = safe_pos[0]
                    trajectory_msg.Rknee_pos_ref = safe_pos[1]
                    trajectory_msg.Rhip_vel_ref = 0.0
                    trajectory_msg.Rknee_vel_ref = 0.0
                    trajectory_msg.Lhip_pos_ref = safe_pos[0]
                    trajectory_msg.Lknee_pos_ref = safe_pos[1]
                    trajectory_msg.Lhip_vel_ref = 0.0
                    trajectory_msg.Lknee_vel_ref = 0.0
            else:
                # No active trajectory - use safe values
                trajectory_msg.Rhip_pos_ref = safe_pos[0]
                trajectory_msg.Rknee_pos_ref = safe_pos[1]
                trajectory_msg.Rhip_vel_ref = 0.0
                trajectory_msg.Rknee_vel_ref = 0.0
                trajectory_msg.Lhip_pos_ref = safe_pos[0]
                trajectory_msg.Lknee_pos_ref = safe_pos[1]
                trajectory_msg.Lhip_vel_ref = 0.0
                trajectory_msg.Lknee_vel_ref = 0.0
        else:
            # All other states: use safe values
            trajectory_msg.Rhip_pos_ref = safe_pos[0]
            trajectory_msg.Rknee_pos_ref = safe_pos[1]
            trajectory_msg.Rhip_vel_ref = 0.0
            trajectory_msg.Rknee_vel_ref = 0.0
            trajectory_msg.Lhip_pos_ref = safe_pos[0]
            trajectory_msg.Lknee_pos_ref = safe_pos[1]
            trajectory_msg.Lhip_vel_ref = 0.0
            trajectory_msg.Lknee_vel_ref = 0.0
        
        # Publish trajectory
        self.joints_trajectory_pub.publish(trajectory_msg)

    def trigger_emergency_stop_and_shutdown(self, reason="GMR trajectory generator emergency"):
        """Trigger emergency stop and shutdown the node."""
        rospy.logerr(f"GMR TRAJECTORY GENERATOR EMERGENCY: {reason}")
        
        # Set emergency stop flag
        self.is_emergency_stop = True
        self.trajectory_active = False
        
        # Send emergency stop message
        e_stop_msg = EStopTrigger()
        e_stop_msg.header.stamp = rospy.Time.now()
        e_stop_msg.trigger = True
        e_stop_msg.state = "GMR_TRAJECTORY_EMERGENCY"
        self.e_stop_trigger_pub.publish(e_stop_msg)
        
        # Shutdown after brief delay
        rospy.Timer(rospy.Duration(0.5), lambda event: rospy.signal_shutdown(reason), oneshot=True)

    def run(self):
        """Main execution loop."""
        rospy.loginfo("gmr: GMR Trajectory Generator Node running...")
        
        while not rospy.is_shutdown():
            try:
                # State-based logic following the emergency stop node state machine
                if self.system_state == "INIT":
                    # In INIT state - wait for system to be ready
                    rospy.loginfo_throttle(10, "gmr: GMR trajectory generator in INIT state - waiting")
                    
                elif self.system_state == "CALIBRATION_PROCESS":
                    # In CALIBRATION_PROCESS state - wait for calibration to complete
                    rospy.loginfo_throttle(5, "gmr: GMR trajectory generator in CALIBRATION_PROCESS state - waiting")
                    
                elif self.system_state == "READY":
                    # In READY state - publish safe values, wait for WALKING state
                    if not self.is_emergency_stop:
                        self.publish_trajectory()
                    
                elif self.system_state == "WALKING":
                    # In WALKING state - generate and publish GMR trajectory
                    if not self.is_emergency_stop:
                        if not self.trajectory_active:
                            self.start_trajectory()
                        
                        self.publish_trajectory()
                        
                        # Update time for next iteration
                        if self.trajectory_active:
                            dt = 1.0 / self.control_frequency
                            self.current_time += dt
                            
                            # Check if cycle completed for looping
                            if self.current_time >= self.trajectory_duration:
                                if self.loop_trajectory and self.system_state == "WALKING":
                                    self.current_time = 0.0  # Restart cycle
                                    rospy.loginfo("gmr: GMR trajectory cycle completed, restarting")
                        
                elif self.system_state == "STOPPING":
                    # In STOPPING state - continue current trajectory until cycle ends
                    if not self.is_emergency_stop:
                        self.publish_trajectory()
                        
                        # Update time and check for cycle completion
                        if self.trajectory_active:
                            dt = 1.0 / self.control_frequency
                            self.current_time += dt
                            
                            # Check if cycle completed - signal finished
                            if self.current_time >= self.trajectory_duration:
                                rospy.loginfo("gmr: GMR trajectory completed in STOPPING state")
                                self.trajectory_active = False
                                self.send_cycle_finished()
                                rospy.loginfo("gmr: Sent cycle finished signal")
                        
                elif self.system_state == "E_STOP":
                    # Emergency stop state - should not reach here
                    rospy.logerr("gmr: GMR trajectory generator in E_STOP state - shutting down")
                    break
                    
                else:
                    rospy.logwarn_throttle(5, f"gmr: Unknown system state: {self.system_state}")
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"gmr: Error in GMR trajectory generator loop: {e}")
                self.rate.sleep()

    def shutdown(self):
        """Clean shutdown method called on node shutdown."""
        rospy.loginfo("gmr: Shutting down GMR Trajectory Generator Node...")
        
        # Stop trajectory playback
        self.trajectory_active = False
        
        if not self.is_emergency_stop:
            rospy.loginfo("gmr: Clean shutdown - GMR trajectory generator stopping normally")
        else:
            rospy.loginfo("gmr: Emergency shutdown already performed")
        
        rospy.loginfo("gmr: GMR Trajectory Generator Node shutdown complete")

if __name__ == '__main__':
    try:
        node = GMRTrajectoryGeneratorNode()
        
        # Register shutdown callback
        rospy.on_shutdown(node.shutdown)
        
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("gmr: GMR Trajectory Generator Node shutdown")
    except Exception as e:
        rospy.logerr(f"gmr: Unexpected error in GMR Trajectory Generator Node: {e}")