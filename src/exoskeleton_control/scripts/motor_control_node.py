#!/usr/bin/env python3

import rospy
import numpy as np
import threading
import time
import math
from enum import Enum
from exoskeleton_control.msg import JointsTrajectory, ExoskeletonState, MotorStatus, Torques, EStopTrigger, CalibrationTrigger
import MIT_motor_controller_v2 as motor_driver
import candle_driver

class CalibrationState(Enum):
    """Calibration state machine states"""
    NOT_STARTED = "not_started"
    CALIBRATING_R_HIP = "calibrating_r_hip"
    CALIBRATING_L_HIP = "calibrating_l_hip" 
    CALIBRATING_R_KNEE = "calibrating_r_knee"
    CALIBRATING_L_KNEE = "calibrating_l_knee"
    COMPLETED = "completed"
    FAILED = "failed"

class MotorConfig:
    """Motor configuration data"""
    def __init__(self, motor_id, model, controller_id, joint_name, min_angle_deg, max_angle_deg):
        self.motor_id = motor_id
        self.model = model
        self.controller_id = controller_id
        self.joint_name = joint_name
        self.min_angle_rad = math.radians(min_angle_deg)
        self.max_angle_rad = math.radians(max_angle_deg)
        self.direction = 1  # Will be set from configuration
        self.is_calibrated = False
        self.min_limit = 0.0  # Will be set during calibration
        self.max_limit = 0.0  # Will be set during calibration

class FeedforwardTorqueCalculator:
    """
    Fast feedforward torque calculator for 2-link planar arm control system.
    Optimized for ROS1 node integration.
    """

    def __init__(self, dt: float = 0.01, alpha: float = 0.1):
        """
        Initialize the feedforward torque calculator.

        Args:
            dt: Sample time for acceleration estimation [s]
            alpha: Low-pass filter coefficient for acceleration (0-1)
        """
        # Physical parameters (same as MATLAB)
        self.m1 = 8.0      # Link 1 mass [kg]
        self.m2 = 3.7      # Link 2 mass [kg] 
        self.l1 = 0.44     # Link 1 length [m]
        self.lc1 = 0.2     # Link 1 center of mass [m]
        self.lc2 = 0.2     # Link 2 center of mass [m]
        self.I1 = 0.13     # Link 1 inertia [kg⋅m²]
        self.I2 = 0.05     # Link 2 inertia [kg⋅m²]
        self.g = 9.81      # Gravity [m/s²]

        # Acceleration estimation parameters
        self.dt = dt
        self.alpha = alpha  # Low-pass filter coefficient

        # State history for acceleration estimation
        self.prev_q_dot = None
        self.prev_time = None
        self.q_ddot_filtered = np.zeros(2)

        # Pre-allocate arrays for speed
        self.M_matrix = np.zeros((2, 2))
        self.G_vector = np.zeros(2)
        self.Cqd_vector = np.zeros(2)

    def calculate_feedforward_torque(self, q, qdot, tau):
        """
        Calculate feedforward torque for 2-link arm.

        Args:
            q: Current joint positions [rad] (2x1)
            qdot: Current joint velocities [rad/s] (2x1)  
            tau: Current motor torques [N⋅m] (2x1)

        Returns:
            tau_ff: Feedforward torques [N⋅m] (2x1)
        """
        # Ensure inputs are numpy arrays
        q = np.asarray(q, dtype=np.float64)
        qdot = np.asarray(qdot, dtype=np.float64)
        tau = np.asarray(tau, dtype=np.float64)

        # Get current time
        current_time = rospy.Time.now().to_sec()

        # Estimate acceleration
        q_ddot = self._estimate_acceleration(qdot, current_time)

        # Extract joint states for readability
        q1, q2 = q[0], q[1]
        q1d, q2d = qdot[0], qdot[1]
        q1dd, q2dd = q_ddot[0], q_ddot[1]

        # Pre-compute trigonometric functions
        sin_q1 = np.sin(q1)
        sin_q2 = np.sin(q2)
        cos_q2 = np.cos(q2)
        sin_q1_q2 = np.sin(q1 + q2)

        # Mass Matrix M(q) - optimized computation
        m2_half = self.m2 * 0.5
        lc2_sq = self.lc2**2
        l1_lc2_cos = self.l1 * self.lc2 * cos_q2

        self.M_matrix[0, 0] = (self.m1 * self.lc1**2 + self.I1 + self.I2 + 
                              m2_half * (2 * self.l1**2 + 4 * l1_lc2_cos + 2 * lc2_sq))
        self.M_matrix[0, 1] = self.I2 + m2_half * (2 * lc2_sq + 2 * l1_lc2_cos)
        self.M_matrix[1, 0] = self.M_matrix[0, 1]  # Symmetric
        self.M_matrix[1, 1] = self.m2 * lc2_sq + self.I2

        # Gravity Vector G(q)
        self.G_vector[0] = (self.g * self.m2 * (self.lc2 * sin_q1_q2 + self.l1 * sin_q1) + 
                           self.g * self.lc1 * self.m1 * sin_q1)
        self.G_vector[1] = self.g * self.lc2 * self.m2 * sin_q1_q2

        # Coriolis and Centrifugal Vector C(q, qdot)*qdot
        l1_lc2_m2_sin_q2 = self.l1 * self.lc2 * self.m2 * sin_q2
        self.Cqd_vector[0] = -l1_lc2_m2_sin_q2 * q2d * (2 * q1d + q2d)
        self.Cqd_vector[1] = l1_lc2_m2_sin_q2 * q1d**2

        # Feedforward torque calculation
        # tau_ff = M(q)*q_ddot + C(q,qdot)*qdot + G(q)
        tau_ff = (np.dot(self.M_matrix, q_ddot) + 
                 self.Cqd_vector + 
                 self.G_vector)

        return tau_ff

    def _estimate_acceleration(self, qdot, current_time):
        """
        Estimate acceleration using filtered finite differences.
        """
        if self.prev_q_dot is None or self.prev_time is None:
            # First call - initialize
            self.prev_q_dot = qdot.copy()
            self.prev_time = current_time
            return np.zeros(2)

        # Calculate dt
        dt = current_time - self.prev_time
        if dt <= 0 or dt > 0.1:  # Protect against invalid dt
            dt = self.dt

        # Finite difference acceleration
        q_ddot_raw = (qdot - self.prev_q_dot) / dt

        # Low-pass filter to smooth acceleration
        self.q_ddot_filtered = (self.alpha * q_ddot_raw + 
                               (1 - self.alpha) * self.q_ddot_filtered)

        # Update history
        self.prev_q_dot = qdot.copy()
        self.prev_time = current_time

        return self.q_ddot_filtered.copy()

    def reset_estimator(self):
        """Reset the acceleration estimator state."""
        self.prev_q_dot = None
        self.prev_time = None
        self.q_ddot_filtered = np.zeros(2)


class MotorControlNode:
    def __init__(self):
        rospy.init_node('motor_control_node')

        # Load configuration parameters
        self.load_configuration()
        
        # Control parameters
        self.control_frequency = 100.0  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # State management
        self.calibration_state = CalibrationState.NOT_STARTED
        self.is_emergency_stop = False
        self.trajectory_active = False
        
        # Initialize motor configurations from parameters
        self.setup_motor_configurations()
        
        # Current calibration motor index
        self.current_calibration_motor = 0
        
        # Motor controllers and states
        self.motor_controllers = {}
        self.motor_states = {}
        self.can_channel = None
        
        # Motor state storage (ordered by motor ID)
        self.num_motors = len(self.motor_configs)
        self.motor_positions = np.zeros(self.num_motors)
        self.motor_velocities = np.zeros(self.num_motors)
        self.motor_torques = np.zeros(self.num_motors)
        self.motor_temperatures = np.zeros(self.num_motors)
        self.motor_error_flags = np.zeros(self.num_motors, dtype=int)

        # Desired trajectory storage
        self.desired_positions = np.zeros(self.num_motors)
        self.desired_velocities = np.zeros(self.num_motors)

        # Feedforward torque calculators (one for each leg)
        self.ff_calc_right = FeedforwardTorqueCalculator(dt=self.dt, alpha=0.1)
        self.ff_calc_left = FeedforwardTorqueCalculator(dt=self.dt, alpha=0.1)

        # Calculated feedforward torques
        self.feedforward_torques = np.zeros(self.num_motors)

        # Threading lock for motor communication
        self.motor_lock = threading.Lock()

        # Initialize CAN interface first
        if not self.initialize_can_interface():
            rospy.signal_shutdown("Failed to initialize CAN interface")
            return

        # Initialize motors
        if not self.initialize_motors():
            rospy.signal_shutdown("Failed to initialize motors")
            return

        # Subscribers
        rospy.Subscriber('joints_trajectory', JointsTrajectory, self.joints_trajectory_callback)
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)
        rospy.Subscriber('calibration_trigger', CalibrationTrigger, self.calibration_trigger_callback)

        # Publishers
        self.exoskeleton_state_pub = rospy.Publisher('ExoskeletonState', ExoskeletonState, queue_size=1)
        self.motor_status_pub = rospy.Publisher('MotorStatus', MotorStatus, queue_size=1)
        self.torques_pub = rospy.Publisher('Torques', Torques, queue_size=1)

        self.rate = rospy.Rate(self.control_frequency)

        rospy.loginfo("Motor Control Node initialized. Waiting for calibration...")
        rospy.loginfo(f"Configuration loaded:")
        rospy.loginfo(f"  Calibration sequence: {self.calibration_sequence}")
        rospy.loginfo(f"  Torque threshold: {self.torque_threshold} Nm")
        rospy.loginfo(f"  Calibration speed: {math.degrees(self.calibration_speed):.1f} deg/s")
        rospy.loginfo(f"  Motor IDs: {list(self.motor_configs.keys())}")

    def load_configuration(self):
        """Load configuration parameters from ROS parameter server."""
        try:
            # Calibration parameters
            self.calibration_sequence = rospy.get_param('~calibration_sequence', [1, 3, 2, 4])
            self.torque_threshold = rospy.get_param('~torque_threshold', 8.0)
            self.calibration_speed = math.radians(rospy.get_param('~calibration_speed_deg', 30.0))
            self.max_calibration_time = rospy.get_param('~max_calibration_time', 30.0)

            # Control gains
            self.gains = {
                'calibration': {
                    'kp': rospy.get_param('~gains/calibration/kp', 10.0),
                    'kd': rospy.get_param('~gains/calibration/kd', 1.0)
                },
                'hold': {
                    'kp': rospy.get_param('~gains/hold/kp', 5.0),
                    'kd': rospy.get_param('~gains/hold/kd', 1.0)
                },
                'trajectory': {
                    'kp': rospy.get_param('~gains/trajectory/kp', 50.0),
                    'kd': rospy.get_param('~gains/trajectory/kd', 2.0)
                }
            }

            # Motor configurations
            self.motor_config_params = {
                'motor_ids': rospy.get_param('~motor_ids', [0x06, 0x07, 0x08, 0x09]),
                'joint_names': rospy.get_param('~joint_names', ["right_hip", "right_knee", "left_hip", "left_knee"]),
                'motor_models': rospy.get_param('~motor_models', ["AK80_64", "AK80_8", "AK80_64", "AK80_8"]),
                'angle_limits_deg': rospy.get_param('~angle_limits_deg', [[-30, 90], [-100, 0], [-30, 90], [-100, 0]]),
                'motor_directions': rospy.get_param('~motor_directions', [1, 1, 1, 1])
            }

            rospy.loginfo("Configuration parameters loaded successfully from parameter server")

        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            rospy.logerr("Using default configuration values")
            self.set_default_configuration()

    def set_default_configuration(self):
        """Set default configuration values if parameter loading fails."""
        # Calibration parameters
        self.calibration_sequence = [1, 3, 2, 4]
        self.torque_threshold = 8.0
        self.calibration_speed = math.radians(30.0)
        self.max_calibration_time = 30.0

        # Control gains
        self.gains = {
            'calibration': {'kp': 10.0, 'kd': 1.0},
            'hold': {'kp': 5.0, 'kd': 1.0},
            'trajectory': {'kp': 50.0, 'kd': 2.0}
        }

        # Motor configurations
        self.motor_config_params = {
            'motor_ids': [0x06, 0x07, 0x08, 0x09],
            'joint_names': ["right_hip", "right_knee", "left_hip", "left_knee"],
            'motor_models': ["AK80_64", "AK80_8", "AK80_64", "AK80_8"],
            'angle_limits_deg': [[-30, 90], [-100, 0], [-30, 90], [-100, 0]],
            'motor_directions': [1, 1, 1, 1]
        }

    def setup_motor_configurations(self):
        """Setup motor configurations from loaded parameters."""
        try:
            self.motor_configs = {}
            
            # Map string motor models to enum values
            model_map = {
                "AK10_9": motor_driver.MotorModel.AK10_9,
                "AK60_6": motor_driver.MotorModel.AK60_6,
                "AK70_10": motor_driver.MotorModel.AK70_10,
                "AK80_6": motor_driver.MotorModel.AK80_6,
                "AK80_9": motor_driver.MotorModel.AK80_9,
                "AK80_64": motor_driver.MotorModel.AK80_64,
                "AK80_8": motor_driver.MotorModel.AK80_8
            }

            for i in range(len(self.motor_config_params['motor_ids'])):
                motor_id = i + 1  # Internal motor ID (1, 2, 3, 4)
                controller_id = self.motor_config_params['motor_ids'][i]  # CAN ID
                joint_name = self.motor_config_params['joint_names'][i]
                model_str = self.motor_config_params['motor_models'][i]
                angle_limits = self.motor_config_params['angle_limits_deg'][i]
                direction = self.motor_config_params['motor_directions'][i]

                # Get motor model enum
                if model_str not in model_map:
                    rospy.logerr(f"Unknown motor model: {model_str}")
                    continue

                motor_model = model_map[model_str]

                # Create motor configuration
                config = MotorConfig(
                    motor_id=motor_id,
                    model=motor_model,
                    controller_id=controller_id,
                    joint_name=joint_name,
                    min_angle_deg=angle_limits[0],
                    max_angle_deg=angle_limits[1]
                )
                
                # Add direction multiplier
                config.direction = direction

                self.motor_configs[motor_id] = config

            rospy.loginfo(f"Motor configurations setup complete: {len(self.motor_configs)} motors")

        except Exception as e:
            rospy.logerr(f"Error setting up motor configurations: {e}")
            rospy.signal_shutdown("Failed to setup motor configurations")

    def initialize_can_interface(self):
        """Initialize CAN interface using candle_driver."""
        try:
            # List available CAN devices
            devices = candle_driver.list_devices()
            if not devices:
                rospy.logerr('No candle devices found.')
                return False

            # Use first available device
            device = devices[0]
            rospy.loginfo(f'Using CAN device: {device.name()}')

            # Open device
            if not device.open():
                rospy.logerr("Failed to open CAN device")
                return False

            # Get CAN channel
            self.can_channel = device.channel(0)
            self.can_channel.set_bitrate(1000000)  # 1 Mbps

            # Start CAN channel
            if not self.can_channel.start():
                rospy.logerr("Failed to start CAN channel")
                device.close()
                return False

            rospy.loginfo("CAN interface initialized successfully!")
            return True

        except Exception as e:
            rospy.logerr(f"Failed to initialize CAN interface: {e}")
            return False

    def initialize_motors(self):
        """Initialize motor controllers."""
        try:
            for motor_id, config in self.motor_configs.items():
                # Create motor controller
                self.motor_controllers[motor_id] = motor_driver.MotorController(
                    model=config.model,
                    controller_id=config.controller_id
                )
                
                # Create motor state
                self.motor_states[motor_id] = motor_driver.MotorState()
                
                rospy.loginfo(f"Motor {motor_id} ({config.joint_name}) initialized")

            return True

        except Exception as e:
            rospy.logerr(f"Failed to initialize motors: {e}")
            return False

    def joints_trajectory_callback(self, msg):
        """Process joint trajectory commands."""
        if self.calibration_state != CalibrationState.COMPLETED:
            rospy.logwarn("Cannot execute trajectory - motors not calibrated")
            return

        if self.is_emergency_stop:
            rospy.logwarn("Cannot execute trajectory - emergency stop active")
            return

        # Update desired positions and velocities
        # Expected order: [R_hip, R_knee, L_hip, L_knee]
        if hasattr(msg, 'right_leg') and hasattr(msg, 'left_leg'):
            if len(msg.right_leg.positions) >= 2:
                self.desired_positions[0] = msg.right_leg.positions[0]  # R_hip
                self.desired_positions[1] = msg.right_leg.positions[1]  # R_knee
            if len(msg.right_leg.velocities) >= 2:
                self.desired_velocities[0] = msg.right_leg.velocities[0]  # R_hip
                self.desired_velocities[1] = msg.right_leg.velocities[1]  # R_knee
                
            if len(msg.left_leg.positions) >= 2:
                self.desired_positions[2] = msg.left_leg.positions[0]  # L_hip
                self.desired_positions[3] = msg.left_leg.positions[1]  # L_knee
            if len(msg.left_leg.velocities) >= 2:
                self.desired_velocities[2] = msg.left_leg.velocities[0]  # L_hip
                self.desired_velocities[3] = msg.left_leg.velocities[1]  # L_knee

        # Enforce joint limits
        for i, motor_id in enumerate([1, 2, 3, 4]):
            config = self.motor_configs[motor_id]
            self.desired_positions[i] = np.clip(
                self.desired_positions[i], 
                config.min_angle_rad, 
                config.max_angle_rad
            )

        self.trajectory_active = True

    def e_stop_callback(self, msg):
        """Handle emergency stop."""
        rospy.loginfo(f"Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            self.is_emergency_stop = True
            self.trajectory_active = False
            # Send zero torque commands to all motors
            self.emergency_stop_motors()
            
            # If calibration was in progress, reset to failed state
            if self.calibration_state not in [CalibrationState.NOT_STARTED, CalibrationState.COMPLETED]:
                rospy.logwarn("Emergency stop during calibration - resetting to init state")
                self.reset_calibration()
        else:
            self.is_emergency_stop = False

    def calibration_trigger_callback(self, msg):
        """Handle calibration trigger."""
        if msg.trigger and self.calibration_state == CalibrationState.NOT_STARTED:
            rospy.loginfo("Starting motor calibration...")
            self.start_calibration()

    def start_calibration(self):
        """Start the calibration process."""
        try:
            self.calibration_state = CalibrationState.CALIBRATING_R_HIP
            self.current_calibration_motor = 0
            
            # Reset all motor calibration flags
            for config in self.motor_configs.values():
                config.is_calibrated = False
                
            # Reset feedforward calculators
            self.ff_calc_right.reset_estimator()
            self.ff_calc_left.reset_estimator()
            
            # Enter MIT mode for all motors
            for motor_id, controller in self.motor_controllers.items():
                motor_driver.enter_mode(self.can_channel, controller.controller_id)
                time.sleep(0.1)
                
            rospy.loginfo("All motors entered MIT mode. Starting calibration sequence...")

        except Exception as e:
            rospy.logerr(f"Failed to start calibration: {e}")
            self.calibration_state = CalibrationState.FAILED

    def perform_single_motor_calibration(self, motor_id):
        """Perform calibration for a single motor."""
        rospy.loginfo(f"Calibrating motor {motor_id} ({self.motor_configs[motor_id].joint_name})")
        
        try:
            controller = self.motor_controllers[motor_id]
            state = self.motor_states[motor_id]
            config = self.motor_configs[motor_id]
            
            # Zero the current position
            motor_driver.zero_position(self.can_channel, controller.controller_id)
            time.sleep(0.2)
            
            limits = []
            calibration_start_time = time.time()
            
            # Calibrate in both directions
            for direction in [1, -1]:  # positive first, then negative
                rospy.loginfo(f"  Searching for limit in direction {direction}")
                
                # Reset state for each direction
                state.p_in = 0.0
                state.v_in = direction * self.calibration_speed * config.direction  # Apply motor direction
                state.kp_in = self.gains['calibration']['kp']
                state.kd_in = self.gains['calibration']['kd']
                state.t_in = 0.0    # No feedforward torque
                
                direction_start_time = time.time()
                
                while True:
                    # Check for timeout
                    if time.time() - direction_start_time > self.max_calibration_time:
                        rospy.logerr(f"Calibration timeout for motor {motor_id}")
                        return False
                    
                    # Check for emergency stop
                    if self.is_emergency_stop:
                        rospy.logerr("Emergency stop during calibration")
                        return False
                    
                    # Send movement command
                    if not motor_driver.pack_cmd(self.can_channel, controller, state):
                        rospy.logerr(f"Failed to send calibration command to motor {motor_id}")
                        return False
                    
                    time.sleep(0.01)  # 10ms delay
                    
                    # Read motor status
                    if motor_driver.read_motor_status(self.can_channel, controller, state):
                        # Check if torque exceeds threshold (stopper detected)
                        if abs(state.t_out) > self.torque_threshold:
                            limits.append(state.p_out)
                            rospy.loginfo(f"  Limit found at position: {state.p_out:.3f} rad, torque: {state.t_out:.3f} Nm")
                            
                            # Stop the motor
                            state.v_in = 0.0
                            motor_driver.pack_cmd(self.can_channel, controller, state)
                            time.sleep(0.2)
                            break
                    else:
                        rospy.logwarn(f"Failed to read status from motor {motor_id}")
            
            # Check if both limits were found
            if len(limits) != 2:
                rospy.logerr(f"Failed to find both limits for motor {motor_id}")
                return False
            
            # Store limits (ensure min < max)
            config.min_limit = min(limits)
            config.max_limit = max(limits)
            config.is_calibrated = True
            
            # Move to center position
            center_position = (config.min_limit + config.max_limit) / 2.0
            state.p_in = center_position
            state.v_in = 0.0
            state.kp_in = self.gains['trajectory']['kp']  # Use trajectory gains for positioning
            state.kd_in = self.gains['trajectory']['kd']
            state.t_in = 0.0
            
            motor_driver.pack_cmd(self.can_channel, controller, state)
            time.sleep(1.0)  # Allow time to reach center
            
            rospy.loginfo(f"Motor {motor_id} calibrated successfully:")
            rospy.loginfo(f"  Limits: {config.min_limit:.3f} to {config.max_limit:.3f} rad")
            rospy.loginfo(f"  Range: {math.degrees(config.max_limit - config.min_limit):.1f} degrees")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error calibrating motor {motor_id}: {e}")
            return False

    def update_calibration_state_machine(self):
        """Update the calibration state machine."""
        if self.calibration_state == CalibrationState.NOT_STARTED or self.calibration_state == CalibrationState.COMPLETED:
            return
        
        if self.calibration_state == CalibrationState.FAILED:
            self.reset_calibration()
            return
        
        # Get current motor ID to calibrate
        if self.current_calibration_motor >= len(self.calibration_sequence):
            # All motors calibrated successfully
            self.calibration_state = CalibrationState.COMPLETED
            rospy.loginfo("All motors calibrated successfully!")
            return
        
        motor_id = self.calibration_sequence[self.current_calibration_motor]
        
        # Perform calibration for current motor
        if self.perform_single_motor_calibration(motor_id):
            # Success - move to next motor
            self.current_calibration_motor += 1
            
            # Update state machine
            if self.current_calibration_motor < len(self.calibration_sequence):
                next_motor_id = self.calibration_sequence[self.current_calibration_motor]
                next_joint_name = self.motor_configs[next_motor_id].joint_name
                
                if next_joint_name == "left_hip":
                    self.calibration_state = CalibrationState.CALIBRATING_L_HIP
                elif next_joint_name == "right_knee":
                    self.calibration_state = CalibrationState.CALIBRATING_R_KNEE
                elif next_joint_name == "left_knee":
                    self.calibration_state = CalibrationState.CALIBRATING_L_KNEE
            else:
                self.calibration_state = CalibrationState.COMPLETED
        else:
            # Failure - reset all calibration
            rospy.logerr(f"Calibration failed for motor {motor_id}")
            self.calibration_state = CalibrationState.FAILED

    def reset_calibration(self):
        """Reset calibration to initial state."""
        rospy.loginfo("Resetting calibration to init state")
        self.calibration_state = CalibrationState.NOT_STARTED
        self.current_calibration_motor = 0
        
        # Reset all motor calibration flags
        for config in self.motor_configs.values():
            config.is_calibrated = False
            config.min_limit = 0.0
            config.max_limit = 0.0
        
        # Exit MIT mode for all motors
        try:
            for motor_id, controller in self.motor_controllers.items():
                motor_driver.exit_mode(self.can_channel, controller.controller_id)
                time.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Error exiting MIT mode during reset: {e}")

    def emergency_stop_motors(self):
        """Send zero torque to all motors immediately."""
        try:
            with self.motor_lock:
                for motor_id, controller in self.motor_controllers.items():
                    state = self.motor_states[motor_id]
                    state.p_in = state.p_out  # Hold current position
                    state.v_in = 0.0
                    state.kp_in = 0.0  # Zero position gain
                    state.kd_in = 5.0  # High damping only
                    state.t_in = 0.0   # Zero torque
                    
                    motor_driver.pack_cmd(self.can_channel, controller, state)

        except Exception as e:
            rospy.logerr(f"Emergency stop error: {e}")

    def read_motor_states(self):
        """Read current state from all motors synchronously."""
        try:
            with self.motor_lock:
                for i, motor_id in enumerate([1, 2, 3, 4]):
                    controller = self.motor_controllers[motor_id]
                    state = self.motor_states[motor_id]
                    
                    if motor_driver.read_motor_status(self.can_channel, controller, state):
                        self.motor_positions[i] = state.p_out
                        self.motor_velocities[i] = state.v_out
                        self.motor_torques[i] = state.t_out
                        self.motor_temperatures[i] = state.temperature
                        self.motor_error_flags[i] = state.error_flag
                    else:
                        rospy.logwarn_throttle(1.0, f"Failed to read motor {motor_id} status")

        except Exception as e:
            rospy.logerr(f"Error reading motor states: {e}")

    def calculate_feedforward_torques(self):
        """Calculate feedforward torques for both legs."""
        try:
            # Right leg (motors 1, 2: hip, knee)
            q_right = self.motor_positions[0:2]
            qdot_right = self.motor_velocities[0:2] 
            tau_right = self.motor_torques[0:2]

            tau_ff_right = self.ff_calc_right.calculate_feedforward_torque(
                q_right, qdot_right, tau_right)

            # Left leg (motors 3, 4: hip, knee)
            q_left = self.motor_positions[2:4]
            qdot_left = self.motor_velocities[2:4]
            tau_left = self.motor_torques[2:4]

            tau_ff_left = self.ff_calc_left.calculate_feedforward_torque(
                q_left, qdot_left, tau_left)

            # Store calculated feedforward torques
            self.feedforward_torques[0:2] = tau_ff_right
            self.feedforward_torques[2:4] = tau_ff_left

        except Exception as e:
            rospy.logerr(f"Error calculating feedforward torques: {e}")
            self.feedforward_torques.fill(0.0)

    def send_motor_commands(self):
        """Send impedance control commands to all motors."""
        if self.calibration_state not in [CalibrationState.COMPLETED] or self.is_emergency_stop:
            return

        try:
            with self.motor_lock:
                for i, motor_id in enumerate([1, 2, 3, 4]):
                    controller = self.motor_controllers[motor_id]
                    state = self.motor_states[motor_id]
                    config = self.motor_configs[motor_id]
                    
                    if self.trajectory_active and config.is_calibrated:
                        # Clip desired position to calibrated limits
                        desired_pos = np.clip(
                            self.desired_positions[i],
                            config.min_limit,
                            config.max_limit
                        )
                        
                        # Set impedance control parameters
                        state.p_in = desired_pos
                        state.v_in = self.desired_velocities[i] * config.direction  # Apply motor direction
                        state.kp_in = self.gains['trajectory']['kp']
                        state.kd_in = self.gains['trajectory']['kd']
                        state.t_in = self.feedforward_torques[i] * config.direction  # Apply direction to torque
                        
                        motor_driver.pack_cmd(self.can_channel, controller, state)
                    else:
                        # Send zero torque if no active trajectory
                        state.p_in = state.p_out  # Hold current position
                        state.v_in = 0.0
                        state.kp_in = self.gains['hold']['kp']
                        state.kd_in = self.gains['hold']['kd']
                        state.t_in = 0.0
                        
                        motor_driver.pack_cmd(self.can_channel, controller, state)

        except Exception as e:
            rospy.logerr(f"Error sending motor commands: {e}")

    def publish_exoskeleton_state(self):
        """Publish current exoskeleton state."""
        try:
            msg = ExoskeletonState()
            msg.header.stamp = rospy.Time.now()
            
            # Right leg data
            msg.right_leg.positions = [self.motor_positions[0], self.motor_positions[1]]
            msg.right_leg.velocities = [self.motor_velocities[0], self.motor_velocities[1]]
            msg.right_leg.torques = [self.motor_torques[0], self.motor_torques[1]]
            
            # Left leg data
            msg.left_leg.positions = [self.motor_positions[2], self.motor_positions[3]]
            msg.left_leg.velocities = [self.motor_velocities[2], self.motor_velocities[3]]
            msg.left_leg.torques = [self.motor_torques[2], self.motor_torques[3]]
            
            # System state
            msg.is_calibrated = (self.calibration_state == CalibrationState.COMPLETED)
            msg.calibration_state = self.calibration_state.value
            msg.is_emergency_stop = self.is_emergency_stop
            msg.trajectory_active = self.trajectory_active

            self.exoskeleton_state_pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error publishing exoskeleton state: {e}")

    def publish_motor_status(self):
        """Publish individual motor status."""
        try:
            msg = MotorStatus()
            msg.header.stamp = rospy.Time.now()
            
            for i, motor_id in enumerate([1, 2, 3, 4]):
                motor_msg = MotorStatus()  # Create individual motor status
                motor_msg.joint_name = self.motor_configs[motor_id].joint_name
                motor_msg.motor_id = motor_id
                motor_msg.calibrated_flag = self.motor_configs[motor_id].is_calibrated
                motor_msg.position = self.motor_positions[i]
                motor_msg.velocity = self.motor_velocities[i]
                motor_msg.torque = self.motor_torques[i]
                motor_msg.temperature = self.motor_temperatures[i]
                motor_msg.error_flags = int(self.motor_error_flags[i])

            self.motor_status_pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error publishing motor status: {e}")

    def publish_torques(self):
        """Publish calculated torques."""
        try:
            msg = Torques()
            msg.header.stamp = rospy.Time.now()
            msg.feedforward_torques = self.feedforward_torques.tolist()
            msg.motor_torques = self.motor_torques.tolist()

            self.torques_pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"Error publishing torques: {e}")

    def run(self):
        """Main control loop running at 100Hz."""
        rospy.loginfo("Starting motor control loop...")

        while not rospy.is_shutdown():
            loop_start_time = time.time()

            try:
                # Handle calibration state machine
                if self.calibration_state not in [CalibrationState.NOT_STARTED, CalibrationState.COMPLETED]:
                    self.update_calibration_state_machine()
                    time.sleep(0.1)  # Slower rate during calibration
                    continue

                # Normal operation
                if self.calibration_state == CalibrationState.COMPLETED:
                    # Step 1: Read motor states (should take ~444μs for 4 motors)
                    self.read_motor_states()

                    # Step 2: Calculate feedforward torques (designed for <8ms)
                    self.calculate_feedforward_torques()

                    # Step 3: Send motor commands (should take ~444μs for 4 motors)
                    self.send_motor_commands()

                # Step 4: Publish topics
                self.publish_exoskeleton_state()
                self.publish_motor_status()
                self.publish_torques()

                # Check loop timing
                loop_time = time.time() - loop_start_time
                if loop_time > 0.008:  # 8ms calculation budget exceeded
                    rospy.logwarn(f"Control loop exceeded time budget: {loop_time*1000:.2f}ms")

            except Exception as e:
                rospy.logerr(f"Error in control loop: {e}")

            self.rate.sleep()

    def shutdown(self):
        """Clean shutdown of motors."""
        rospy.loginfo("Shutting down motor control node...")

        # Send zero torque to all motors
        self.emergency_stop_motors()

        # Exit MIT mode for all motors
        try:
            for motor_id, controller in self.motor_controllers.items():
                motor_driver.exit_mode(self.can_channel, controller.controller_id)
                time.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")

        # Close CAN interface
        if self.can_channel:
            self.can_channel.stop()
        
        rospy.loginfo("Motor control node shutdown complete")


if __name__ == '__main__':
    try:
        node = MotorControlNode()

        # Register shutdown callback
        rospy.on_shutdown(node.shutdown)

        # Start the main control loop
        node.run()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
