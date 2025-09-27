#!/usr/bin/env python3

import rospy
import numpy as np
import math
import pickle
import os
import sys
import json
from scipy import interpolate
from exoskeleton_control.msg import GaitParams, JointsTrajectory, DualAnkleTrajectory, EStopTrigger, Trigger, FSMState

# Add TaskParameterizedGaussianMixtureModels to Python path
sys.path.append('TaskParameterizedGaussianMixtureModels')

class TPGMMTrajectoryGeneratorNode:
    def __init__(self):
        rospy.init_node('tpgmm_trajectory_generator_node')

        # Load configuration parameters
        self.load_configuration()

        # Trajectory state
        self.is_emergency_stop = False
        self.trajectory_active = False
        self.current_time = 0.0
        self.trajectory_duration = 1.0  # Duration of one gait cycle (will be updated from gait analysis data)
        
        # TPGMM model for trajectory generation
        self.tpgmm_model = None
        self.model_data = None
        
        # Gait analysis timing parameters
        self.gait_analysis_data = None
        self.velocity_multiplier = 1.0
        self.interpolation_points = 200
        self.original_mean_duration_s = 1.0
        self.scaled_duration_s = 1.0
        self.interpolated_time_points = None
        
        # System state tracking (synchronized with emergency stop node)
        self.system_state = "INIT"
        self.previous_state = "INIT"

        # Subscribers
        rospy.Subscriber('gait_params', GaitParams, self.gait_params_callback)
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)
        rospy.Subscriber('fsm_state', FSMState, self.fsm_state_callback)

        # Publishers
        self.joints_trajectory_pub = rospy.Publisher('joints_trajectory', JointsTrajectory, queue_size=10)
        self.dual_ankle_trajectory_pub = rospy.Publisher('dual_ankle_trajectory', DualAnkleTrajectory, queue_size=10)
        self.cycle_finished_pub = rospy.Publisher('cycle_finished', Trigger, queue_size=1)
        self.e_stop_trigger_pub = rospy.Publisher('e_stop_trigger', EStopTrigger, queue_size=1)

        # Load TPGMM model
        self.load_tpgmm_model()
        
        # Load gait analysis data
        self.load_gait_analysis_data()

        # Control rate
        self.rate = rospy.Rate(self.control_frequency)

        rospy.loginfo("tpgmm: TPGMM Trajectory Generator Node initialized")
        if self.tpgmm_model:
            rospy.loginfo(f"tpgmm: Loaded TPGMM model with {self.model_data['n_components']} components")
        else:
            rospy.logwarn("tpgmm: No TPGMM model loaded - waiting for model file")
        
        if self.gait_analysis_data:
            rospy.loginfo(f"tpgmm: Trajectory timing configured for {self.scaled_duration_s:.3f}s cycles at {self.control_frequency}Hz")
        else:
            rospy.logwarn("tpgmm: No gait analysis data loaded - using default timing")

    def load_configuration(self):
        """Load configuration parameters from ROS parameter server."""
        try:
            # Control parameters
            self.control_frequency = rospy.get_param('~control_frequency', 100)
            
            # Leg parameters
            self.L1 = rospy.get_param('~leg_parameters/L1', 0.425)  # Thigh length (m)
            self.L2 = rospy.get_param('~leg_parameters/L2', 0.45)   # Shin length (m)

            # Joint limits (in radians)
            self.theta1_min = math.radians(rospy.get_param('~joint_limits/hip_min_deg', -10))
            self.theta1_max = math.radians(rospy.get_param('~joint_limits/hip_max_deg', 50))
            self.theta2_min = math.radians(rospy.get_param('~joint_limits/knee_min_deg', -60))
            self.theta2_max = math.radians(rospy.get_param('~joint_limits/knee_max_deg', 0))
            
            # TPGMM model file path
            self.model_file = rospy.get_param('~model_file', 'src/exoskeleton_control/pkls/gait_tpgmm_model_final.pkl')
            
            # Gait analysis data file path
            self.gait_analysis_file = rospy.get_param('~gait_analysis_file', 'src/exoskeleton_control/data/4D/gait_analysis_export_subject35.json')
            
            # Trajectory parameters
            self.trajectory_scale = rospy.get_param('~trajectory_scale', 1.0)
            self.loop_trajectory = rospy.get_param('~loop_trajectory', True)
            
            # Frame index to use (default to first frame)
            self.frame_idx = rospy.get_param('~frame_idx', 0)
            
            # Gait timing configuration
            self.use_gait_analysis_timing = rospy.get_param('~gait_timing/use_gait_analysis_timing', True)
            self.fallback_duration = rospy.get_param('~gait_timing/fallback_duration', 1.0)
            self.interpolation_method = rospy.get_param('~gait_timing/interpolation_method', 'linear')
            
            # Error codes
            self.ERROR_UNREACHABLE = -0.123
            self.ERROR_JOINT_LIMITS = -0.321

            rospy.loginfo("tpgmm: TPGMM trajectory generator configuration loaded successfully")

        except Exception as e:
            rospy.logerr(f"tpgmm: Error loading configuration: {e}")
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
        self.model_file = 'src/exoskeleton_control/pkls/gait_tpgmm_model_final.pkl'
        self.gait_analysis_file = 'src/exoskeleton_control/data/4D/gait_analysis_export_subject35.json'
        self.trajectory_scale = 1.0
        self.loop_trajectory = True
        self.frame_idx = 0
        self.use_gait_analysis_timing = True
        self.fallback_duration = 1.0
        self.interpolation_method = 'linear'
        self.ERROR_UNREACHABLE = -0.123
        self.ERROR_JOINT_LIMITS = -0.321

    def load_tpgmm_model(self):
        """Load the trained TPGMM model from file."""
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
                possible_paths.insert(1, os.path.join(package_path, 'pkls', 'gait_tpgmm_model_final.pkl'))
            except:
                pass
            
            file_found = False
            for path in possible_paths:
                if os.path.exists(path):
                    with open(path, 'rb') as f:
                        self.model_data = pickle.load(f)
                    self.tpgmm_model = self.model_data['tpgmm']
                    file_found = True
                    rospy.loginfo(f"tpgmm: TPGMM model loaded from: {path}")
                    break
            
            if not file_found:
                rospy.logerr(f"tpgmm: TPGMM model file not found in any of these locations: {possible_paths}")
                return False

            # Validate model structure
            required_fields = ['tpgmm', 'feature_names', 'n_components']
            for field in required_fields:
                if field not in self.model_data:
                    rospy.logerr(f"tpgmm: Missing required field in model: {field}")
                    return False

            rospy.loginfo(f"tpgmm: Model validation successful")
            rospy.loginfo(f"tpgmm: - Components: {self.model_data['n_components']}")
            rospy.loginfo(f"tpgmm: - Features: {self.model_data['feature_names']}")
            
            return True

        except Exception as e:
            rospy.logerr(f"tpgmm: Error loading TPGMM model: {e}")
            return False

    def load_gait_analysis_data(self):
        """Load gait analysis timing parameters from JSON file."""
        if not self.use_gait_analysis_timing:
            rospy.loginfo("tpgmm: Gait analysis timing disabled - using fallback duration")
            self.trajectory_duration = self.fallback_duration
            return True
            
        try:
            # Try to find the gait analysis file in multiple locations
            possible_paths = [
                self.gait_analysis_file,
                os.path.expanduser(f'~/{self.gait_analysis_file}'),
                os.path.join('/tmp', os.path.basename(self.gait_analysis_file))
            ]
            
            # Try with ROS package path
            try:
                import rospkg
                rospack = rospkg.RosPack()
                package_path = rospack.get_path('exoskeleton_control')
                possible_paths.insert(1, os.path.join(package_path, 'data/4D/gait_analysis_export_subject35.json'))
            except:
                pass
            
            file_found = False
            for path in possible_paths:
                if os.path.exists(path):
                    with open(path, 'r') as f:
                        self.gait_analysis_data = json.load(f)
                    file_found = True
                    rospy.loginfo(f"tpgmm: Gait analysis data loaded from: {path}")
                    break
            
            if not file_found:
                rospy.logwarn(f"tpgmm: Gait analysis file not found in any of these locations: {possible_paths}")
                rospy.logwarn(f"tpgmm: Using fallback duration: {self.fallback_duration}s")
                self.trajectory_duration = self.fallback_duration
                return True

            # Extract timing parameters
            if 'parameters' in self.gait_analysis_data:
                params = self.gait_analysis_data['parameters']
                
                self.velocity_multiplier = params.get('velocity_multiplier', 1.0)
                self.interpolation_points = params.get('interpolation_points', 200)
                self.original_mean_duration_s = params.get('original_mean_duration_s', 1.0)
                self.scaled_duration_s = params.get('scaled_duration_s', 1.0)
                
                # Update trajectory duration based on scaled duration
                self.trajectory_duration = self.scaled_duration_s
                
                # Calculate interpolated time points based on control frequency
                total_points = int(self.scaled_duration_s * self.control_frequency)
                self.interpolated_time_points = np.linspace(0.0, 1.0, total_points)
                
                rospy.loginfo(f"tpgmm: Gait analysis parameters loaded:")
                rospy.loginfo(f"tpgmm: - Velocity multiplier: {self.velocity_multiplier}")
                rospy.loginfo(f"tpgmm: - Interpolation points: {self.interpolation_points}")
                rospy.loginfo(f"tpgmm: - Original duration: {self.original_mean_duration_s:.3f}s")
                rospy.loginfo(f"tpgmm: - Scaled duration: {self.scaled_duration_s:.3f}s")
                rospy.loginfo(f"tpgmm: - Total interpolated points: {total_points}")
                
                return True
            else:
                rospy.logwarn("tpgmm: Missing 'parameters' section in gait analysis data")
                rospy.logwarn(f"tpgmm: Using fallback duration: {self.fallback_duration}s")
                self.trajectory_duration = self.fallback_duration
                return True

        except Exception as e:
            rospy.logwarn(f"tpgmm: Error loading gait analysis data: {e}")
            rospy.logwarn(f"tpgmm: Using fallback duration: {self.fallback_duration}s")
            self.trajectory_duration = self.fallback_duration
            return True

    def predict_using_tpgmm(self, time_input, feature_idx_to_predict):
        """
        Perform TPGMM-based trajectory prediction to generate ankle trajectories.
        
        Args:
            time_input: Time value (0.0 to 1.0 for normalized gait cycle)
            feature_idx_to_predict: List of feature indices to predict
            
        Returns:
            Dictionary with ankle position and velocity data, or None if error
        """
        if self.tpgmm_model is None:
            rospy.logwarn("tpgmm: No TPGMM model available for prediction")
            return None

        try:
            # Normalize input time to [0, 1] range
            normalized_time = max(0.0, min(1.0, time_input))
            
            # Use the configured frame (default: first frame)
            frame_idx = self.frame_idx
            
            # Get the means and covariances for the selected frame
            means = self.tpgmm_model.means_[frame_idx]  # Shape: (n_components, n_features)
            covariances = self.tpgmm_model.covariances_[frame_idx]  # Shape: (n_components, n_features, n_features)
            weights = self.tpgmm_model.weights_  # Shape: (n_components,)
            
            n_output_features = len(feature_idx_to_predict)
            
            # Time index (input)
            time_idx = 0
            
            # Calculate responsibilities (h) for each component at this time point
            responsibilities = np.zeros(self.tpgmm_model._n_components)
            
            for k in range(self.tpgmm_model._n_components):
                # Simple Gaussian evaluation at time t
                mean_time = means[k, time_idx]
                var_time = covariances[k, time_idx, time_idx]
                
                # Gaussian probability at time t
                prob = np.exp(-0.5 * ((normalized_time - mean_time) ** 2) / var_time) / np.sqrt(2 * np.pi * var_time)
                responsibilities[k] = weights[k] * prob
            
            # Normalize responsibilities
            responsibilities /= (np.sum(responsibilities) + 1e-10)
            
            # Predict output features using Gaussian Mixture Regression
            pred_mean = np.zeros(n_output_features)
            
            for k in range(self.tpgmm_model._n_components):
                # Extract means and covariances for input and output
                mu_i = means[k, time_idx]  # input mean
                mu_o = means[k, feature_idx_to_predict]  # output mean
                
                sigma_ii = covariances[k, time_idx, time_idx]  # input-input covariance
                sigma_io = covariances[k, time_idx, feature_idx_to_predict]  # input-output covariance  
                
                # GMR prediction for component k
                pred_mean_k = mu_o + (sigma_io.T / sigma_ii) * (normalized_time - mu_i)
                
                # Weight by responsibility
                pred_mean += responsibilities[k] * pred_mean_k
            
            # Apply trajectory scaling
            pred_mean *= self.trajectory_scale
            
            return {
                'predictions': pred_mean,
                'time': normalized_time,
                'responsibilities': responsibilities
            }

        except Exception as e:
            rospy.logerr(f"tpgmm: Error in TPGMM prediction: {e}")
            return None

    def interpolate_trajectory_timing(self, time_phase):
        """
        Interpolate trajectory timing based on scaled duration and control frequency.
        
        Args:
            time_phase: Current time phase (0.0 to 1.0 for normalized gait cycle)
            
        Returns:
            interpolated_time: Time value interpolated for the TPGMM model based on original timing
        """
        if self.interpolated_time_points is None:
            # Fallback to original timing if interpolation not available
            return time_phase
        
        try:
            # The TPGMM model expects time normalized to the original interpolation points
            # We need to map our current time phase (based on scaled duration) back to 
            # the original model time scale
            
            # Current time index in the interpolated trajectory
            current_index = int(time_phase * (len(self.interpolated_time_points) - 1))
            current_index = max(0, min(current_index, len(self.interpolated_time_points) - 1))
            
            # Get the corresponding normalized time for the TPGMM model
            # The model was trained with interpolation_points, so we need to map back to that scale
            original_time_scale = time_phase  # Keep the same 0-1 scale for TPGMM model
            
            return original_time_scale
            
        except Exception as e:
            rospy.logwarn(f"tpgmm: Error in trajectory timing interpolation: {e}")
            return time_phase

    def get_interpolated_trajectory_point(self, time_phase):
        """
        Generate trajectory point with proper timing interpolation.
        
        Args:
            time_phase: Time phase from 0.0 to 1.0 based on scaled duration
            
        Returns:
            Dictionary with trajectory data or None if error
        """
        if self.tpgmm_model is None:
            return None
        
        try:
            # Get the properly interpolated time for the TPGMM model
            model_time = self.interpolate_trajectory_timing(time_phase)
            
            # Generate the trajectory point using the interpolated time
            return self.generate_trajectory_point(model_time)
            
        except Exception as e:
            rospy.logerr(f"tpgmm: Error generating interpolated trajectory point: {e}")
            return None

    def get_interpolated_dual_ankle_trajectory_point(self, time_phase):
        """
        Generate dual ankle trajectory point with proper timing interpolation.
        
        Args:
            time_phase: Time phase from 0.0 to 1.0 based on scaled duration
            
        Returns:
            Dictionary with dual ankle trajectory data or None if error
        """
        if self.tpgmm_model is None:
            return None
        
        try:
            # Get the properly interpolated time for the TPGMM model
            model_time = self.interpolate_trajectory_timing(time_phase)
            
            # Generate the dual ankle trajectory point using the interpolated time
            return self.generate_dual_ankle_trajectory_point(model_time)
            
        except Exception as e:
            rospy.logerr(f"tpgmm: Error generating interpolated dual ankle trajectory point: {e}")
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
            rospy.logwarn_throttle(1.0, f"tpgmm: Target unreachable: distance={distance:.3f}, max_reach={max_reach:.3f}")
            return self.ERROR_UNREACHABLE, self.ERROR_UNREACHABLE
        
        if distance < min_reach:
            rospy.logwarn_throttle(1.0, f"tpgmm: Target too close: distance={distance:.3f}, min_reach={min_reach:.3f}")
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
            rospy.logwarn_throttle(5.0, f"tpgmm: Joint limits exceeded for target ({x:.3f}, {y:.3f})")
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
                rospy.logwarn_throttle(1.0, "tpgmm: Near singular configuration, using zero velocities")
                return 0.0, 0.0
            
            # Calculate joint velocities: q_dot = J^(-1) * x_dot
            ankle_vel = np.array([ankle_vx, ankle_vy])
            joint_vel = np.linalg.solve(J, ankle_vel)
            
            return joint_vel[0], joint_vel[1]
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"tpgmm: Error calculating joint velocities: {e}")
            return 0.0, 0.0

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def generate_dual_ankle_trajectory_point(self, time_phase):
        """Generate a dual ankle trajectory point using TPGMM."""
        if self.tpgmm_model is None:
            return None
        
        # Define output features to predict (exclude time)
        # Feature indices: [1, 2, 3, 4, 5, 6, 7, 8] corresponding to:
        # [right_ankle_pos_x, right_ankle_pos_y, right_ankle_vel_x, right_ankle_vel_y,
        #  left_ankle_pos_x, left_ankle_pos_y, left_ankle_vel_x, left_ankle_vel_y]
        output_feature_indices = list(range(1, len(self.model_data['feature_names'])))
        
        # Use TPGMM to predict ankle trajectories
        tpgmm_result = self.predict_using_tpgmm(time_phase, output_feature_indices)
        if tpgmm_result is None:
            return None
        
        predictions = tpgmm_result['predictions']
        
        # Extract predictions for each ankle
        # Assuming feature order: [right_pos_x, right_pos_y, right_vel_x, right_vel_y,
        #                         left_pos_x, left_pos_y, left_vel_x, left_vel_y]
        return {
            'right_ankle_pos': predictions[0:2],    # [x, y] position
            'right_ankle_vel': predictions[2:4],    # [vx, vy] velocity
            'left_ankle_pos': predictions[4:6],     # [x, y] position  
            'left_ankle_vel': predictions[6:8],     # [vx, vy] velocity
            'time': tpgmm_result['time'],
            'responsibilities': tpgmm_result['responsibilities']
        }

    def generate_trajectory_point(self, time_phase):
        """Generate a single trajectory point using TPGMM."""
        if self.tpgmm_model is None:
            return None
        
        # Define output features to predict (exclude time)
        # For single leg: [ankle_pos_x, ankle_pos_y, ankle_vel_x, ankle_vel_y]
        output_feature_indices = [1, 2, 3, 4]
        
        # Use TPGMM to predict ankle trajectory
        tpgmm_result = self.predict_using_tpgmm(time_phase, output_feature_indices)
        if tpgmm_result is None:
            return None
        
        predictions = tpgmm_result['predictions']
        
        # Extract ankle position and velocity
        ankle_x, ankle_y = predictions[0:2]
        ankle_vx, ankle_vy = predictions[2:4]
        
        # Calculate inverse kinematics
        theta_hip, theta_knee = self.calculate_inverse_kinematics(ankle_x, ankle_y)
        
        # Skip if IK failed
        if theta_hip < -4.0 or theta_knee < -4.0:
            rospy.logwarn_throttle(1.0, f"tpgmm: IK failed for ankle position ({ankle_x:.3f}, {ankle_y:.3f})")
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
            'time': tpgmm_result['time']
        }

    def gait_params_callback(self, msg):
        """Process gait parameters - could be used for future parameter adjustment."""
        rospy.loginfo("tpgmm: Received gait_params")
        # TODO: Use gait parameters to modify trajectory generation
        pass

    def e_stop_callback(self, msg):
        """Handle emergency stop."""
        rospy.loginfo(f"tpgmm: Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            self.is_emergency_stop = True
            self.trajectory_active = False
        else:
            self.is_emergency_stop = False

    def fsm_state_callback(self, msg):
        """Handle FSM state updates from emergency stop node."""
        rospy.loginfo_throttle(10.0, f"tpgmm: Received FSM state: {msg.state}")
        self.handle_state_transition(msg.state)

    def handle_state_transition(self, new_state):
        """Handle state transitions and perform appropriate actions."""
        if self.previous_state == new_state:
            return  # No state change
            
        rospy.loginfo(f"tpgmm: State transition: {self.previous_state} -> {new_state}")
        
        # Handle transitions based on current state machine logic
        if new_state == "WALKING" and self.previous_state == "READY":
            # Start trajectory when entering WALKING from READY
            if not self.trajectory_active:
                self.start_trajectory()
                rospy.loginfo("tpgmm: Started TPGMM trajectory for WALKING state")
        elif new_state == "STOPPING" and self.previous_state == "WALKING":
            # Continue trajectory but prepare to signal cycle finished
            rospy.loginfo("tpgmm: Entering STOPPING state - will signal cycle finished when trajectory completes")
        elif new_state == "READY":
            # Stop trajectory when entering READY
            if self.trajectory_active:
                self.stop_trajectory()
                rospy.loginfo("tpgmm: Stopped TPGMM trajectory for READY state")
            
        self.previous_state = self.system_state
        self.system_state = new_state

    def start_trajectory(self):
        """Start trajectory playback."""
        if self.tpgmm_model is None:
            rospy.logwarn("tpgmm: No TPGMM model available - cannot start trajectory")
            return False
        
        if self.is_emergency_stop:
            rospy.logwarn("tpgmm: Cannot start trajectory - emergency stop active")
            return False
        
        self.trajectory_active = True
        self.current_time = 0.0
        rospy.loginfo("tpgmm: TPGMM trajectory playback started")
        return True

    def stop_trajectory(self):
        """Stop trajectory playback."""
        self.trajectory_active = False
        rospy.loginfo("tpgmm: TPGMM trajectory playback stopped")

    def send_cycle_finished(self):
        """Send cycle finished signal to emergency stop node."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.cycle_finished_pub.publish(msg)
        rospy.loginfo("tpgmm: Cycle finished signal sent")

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
            # WALKING/STOPPING states: use TPGMM to generate trajectory
            if self.trajectory_active and self.tpgmm_model:
                # Calculate current phase in gait cycle (0.0 to 1.0)
                time_phase = (self.current_time / self.trajectory_duration) % 1.0
                
                # Generate trajectory point using interpolated TPGMM timing
                traj_point = self.get_interpolated_trajectory_point(time_phase)
                
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
                        rospy.loginfo(f"tpgmm: TPGMM trajectory t={time_phase:.2f}: hip={hip_deg:.1f}°, knee={knee_deg:.1f}°")
                else:
                    # Fallback to safe values if TPGMM fails
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
        
        # Also publish dual ankle trajectory for 4-motor system
        self.publish_dual_ankle_trajectory()

    def publish_dual_ankle_trajectory(self):
        """Publish dual ankle trajectory message based on system state."""
        if self.is_emergency_stop:
            return
        
        # Create trajectory message
        trajectory_msg = DualAnkleTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        
        # Default safe values (zero positions and velocities)
        safe_pos = [0.0, 0.0]
        safe_vel = [0.0, 0.0]
        
        # Set values based on system state
        if self.system_state == "READY":
            # READY state: publish safe position with zero velocities
            trajectory_msg.right_ankle_pos_x = safe_pos[0]
            trajectory_msg.right_ankle_pos_y = safe_pos[1]
            trajectory_msg.right_ankle_vel_x = safe_vel[0]
            trajectory_msg.right_ankle_vel_y = safe_vel[1]
            trajectory_msg.left_ankle_pos_x = safe_pos[0]
            trajectory_msg.left_ankle_pos_y = safe_pos[1]
            trajectory_msg.left_ankle_vel_x = safe_vel[0]
            trajectory_msg.left_ankle_vel_y = safe_vel[1]
            trajectory_msg.time_phase = 0.0
            
        elif self.system_state == "WALKING" or self.system_state == "STOPPING":
            # WALKING/STOPPING states: use TPGMM to generate trajectory
            if self.trajectory_active and self.tpgmm_model:
                # Calculate current phase in gait cycle (0.0 to 1.0)
                time_phase = (self.current_time / self.trajectory_duration) % 1.0
                
                # Generate trajectory point using interpolated TPGMM timing
                traj_point = self.get_interpolated_dual_ankle_trajectory_point(time_phase)
                
                if traj_point is not None:
                    # Use generated trajectory
                    right_pos = traj_point['right_ankle_pos']
                    right_vel = traj_point['right_ankle_vel']
                    left_pos = traj_point['left_ankle_pos']
                    left_vel = traj_point['left_ankle_vel']
                    
                    trajectory_msg.right_ankle_pos_x = right_pos[0]
                    trajectory_msg.right_ankle_pos_y = right_pos[1]
                    trajectory_msg.right_ankle_vel_x = right_vel[0]
                    trajectory_msg.right_ankle_vel_y = right_vel[1]
                    trajectory_msg.left_ankle_pos_x = left_pos[0]
                    trajectory_msg.left_ankle_pos_y = left_pos[1]
                    trajectory_msg.left_ankle_vel_x = left_vel[0]
                    trajectory_msg.left_ankle_vel_y = left_vel[1]
                    trajectory_msg.time_phase = time_phase
                    
                    # Log progress occasionally
                    if int(self.current_time * self.control_frequency) % 50 == 0:
                        rospy.loginfo(f"tpgmm: TPGMM trajectory t={time_phase:.2f}: "
                                    f"R_ankle=({right_pos[0]:.3f}, {right_pos[1]:.3f}), "
                                    f"L_ankle=({left_pos[0]:.3f}, {left_pos[1]:.3f})")
                else:
                    # Fallback to safe values if TPGMM fails
                    trajectory_msg.right_ankle_pos_x = safe_pos[0]
                    trajectory_msg.right_ankle_pos_y = safe_pos[1]
                    trajectory_msg.right_ankle_vel_x = safe_vel[0]
                    trajectory_msg.right_ankle_vel_y = safe_vel[1]
                    trajectory_msg.left_ankle_pos_x = safe_pos[0]
                    trajectory_msg.left_ankle_pos_y = safe_pos[1]
                    trajectory_msg.left_ankle_vel_x = safe_vel[0]
                    trajectory_msg.left_ankle_vel_y = safe_vel[1]
                    trajectory_msg.time_phase = 0.0
            else:
                # No active trajectory - use safe values
                trajectory_msg.right_ankle_pos_x = safe_pos[0]
                trajectory_msg.right_ankle_pos_y = safe_pos[1]
                trajectory_msg.right_ankle_vel_x = safe_vel[0]
                trajectory_msg.right_ankle_vel_y = safe_vel[1]
                trajectory_msg.left_ankle_pos_x = safe_pos[0]
                trajectory_msg.left_ankle_pos_y = safe_pos[1]
                trajectory_msg.left_ankle_vel_x = safe_vel[0]
                trajectory_msg.left_ankle_vel_y = safe_vel[1]
                trajectory_msg.time_phase = 0.0
        else:
            # All other states: use safe values
            trajectory_msg.right_ankle_pos_x = safe_pos[0]
            trajectory_msg.right_ankle_pos_y = safe_pos[1]
            trajectory_msg.right_ankle_vel_x = safe_vel[0]
            trajectory_msg.right_ankle_vel_y = safe_vel[1]
            trajectory_msg.left_ankle_pos_x = safe_pos[0]
            trajectory_msg.left_ankle_pos_y = safe_pos[1]
            trajectory_msg.left_ankle_vel_x = safe_vel[0]
            trajectory_msg.left_ankle_vel_y = safe_vel[1]
            trajectory_msg.time_phase = 0.0
        
        # Publish trajectory
        self.dual_ankle_trajectory_pub.publish(trajectory_msg)

    def trigger_emergency_stop_and_shutdown(self, reason="TPGMM trajectory generator emergency"):
        """Trigger emergency stop and shutdown the node."""
        rospy.logerr(f"TPGMM TRAJECTORY GENERATOR EMERGENCY: {reason}")
        
        # Set emergency stop flag
        self.is_emergency_stop = True
        self.trajectory_active = False
        
        # Send emergency stop message
        e_stop_msg = EStopTrigger()
        e_stop_msg.header.stamp = rospy.Time.now()
        e_stop_msg.trigger = True
        e_stop_msg.state = "TPGMM_TRAJECTORY_EMERGENCY"
        self.e_stop_trigger_pub.publish(e_stop_msg)
        
        # Shutdown after brief delay
        rospy.Timer(rospy.Duration(0.5), lambda event: rospy.signal_shutdown(reason), oneshot=True)

    def run(self):
        """Main execution loop."""
        rospy.loginfo("tpgmm: TPGMM Trajectory Generator Node running...")
        
        while not rospy.is_shutdown():
            try:
                # State-based logic following the emergency stop node state machine
                if self.system_state == "INIT":
                    # In INIT state - wait for system to be ready
                    rospy.loginfo_throttle(10, "tpgmm: TPGMM trajectory generator in INIT state - waiting")
                    
                elif self.system_state == "CALIBRATION_PROCESS":
                    # In CALIBRATION_PROCESS state - wait for calibration to complete
                    rospy.loginfo_throttle(5, "tpgmm: TPGMM trajectory generator in CALIBRATION_PROCESS state - waiting")
                    
                elif self.system_state == "READY":
                    # In READY state - publish safe values, wait for WALKING state
                    if not self.is_emergency_stop:
                        self.publish_trajectory()
                    
                elif self.system_state == "WALKING":
                    # In WALKING state - generate and publish TPGMM trajectory
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
                                    rospy.loginfo("tpgmm: TPGMM trajectory cycle completed, restarting")
                        
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
                                rospy.loginfo("tpgmm: TPGMM trajectory completed in STOPPING state")
                                self.trajectory_active = False
                                self.send_cycle_finished()
                                rospy.loginfo("tpgmm: Sent cycle finished signal")
                        
                elif self.system_state == "E_STOP":
                    # Emergency stop state - should not reach here
                    rospy.logerr("tpgmm: TPGMM trajectory generator in E_STOP state - shutting down")
                    break
                    
                else:
                    rospy.logwarn_throttle(5, f"tpgmm: Unknown system state: {self.system_state}")
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"tpgmm: Error in TPGMM trajectory generator loop: {e}")
                self.rate.sleep()

    def shutdown(self):
        """Clean shutdown method called on node shutdown."""
        rospy.loginfo("tpgmm: Shutting down TPGMM Trajectory Generator Node...")
        
        # Stop trajectory playback
        self.trajectory_active = False
        
        if not self.is_emergency_stop:
            rospy.loginfo("tpgmm: Clean shutdown - TPGMM trajectory generator stopping normally")
        else:
            rospy.loginfo("tpgmm: Emergency shutdown already performed")
        
        rospy.loginfo("tpgmm: TPGMM Trajectory Generator Node shutdown complete")

if __name__ == '__main__':
    try:
        node = TPGMMTrajectoryGeneratorNode()
        
        # Register shutdown callback
        rospy.on_shutdown(node.shutdown)
        
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("tpgmm: TPGMM Trajectory Generator Node shutdown")
    except Exception as e:
        rospy.logerr(f"tpgmm: Unexpected error in TPGMM Trajectory Generator Node: {e}")