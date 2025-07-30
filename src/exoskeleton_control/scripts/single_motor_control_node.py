#!/usr/bin/env python3

import rospy
import numpy as np
import threading
import time
import math
from enum import Enum
from exoskeleton_control.msg import MotorStatus, EStopTrigger, Trigger
import mit_motor_controller as motor_driver
import can

class CalibrationState(Enum):
    """Calibration state machine states"""
    NOT_STARTED = "not_started"
    START_CALIBRATION = "start_calibration"
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
        self.direction = 1
        self.is_calibrated = False
        self.min_limit = 0.0
        self.max_limit = 0.0

class SingleMotorControlNode:
    def __init__(self):
        rospy.init_node('single_motor_control_node')

        self.load_configuration()
        
        self.control_frequency = 100.0  # Hz
        self.dt = 1.0 / self.control_frequency
        
        self.calibration_state = CalibrationState.NOT_STARTED
        self.is_emergency_stop = False
        
        # System state tracking (synchronized with emergency stop node)
        self.system_state = "INIT"  # Track emergency stop node state
        self.previous_state = "INIT"
        self.state_lock = threading.Lock()
        
        self.setup_motor_configuration()
        
        self.motor_controller = None
        self.motor_state = None
        
        self.motor_position = 0.0
        self.motor_velocity = 0.0
        self.motor_torque = 0.0
        self.motor_temperature = 0.0
        self.motor_error_flag = 0

        self.desired_position = 0.0
        self.trajectory = []
        self.trajectory_index = 0

        self.motor_lock = threading.Lock()
        
        # Debug flag for motor driver functions
        self.debug_flag = False

        if not self.initialize_can_interface():
            rospy.signal_shutdown("Failed to initialize CAN interface")
            return

        if not self.initialize_motor():
            rospy.signal_shutdown("Failed to initialize motor")
            return

        # self.generate_trajectory()

        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)
        rospy.Subscriber('system_state', Trigger, self.system_state_callback)

        self.motor_status_pub = rospy.Publisher('Motor_Status', MotorStatus, queue_size=1)
        self.e_stop_trigger_pub = rospy.Publisher('e_stop_trigger', EStopTrigger, queue_size=1)
        self.calibration_failed_pub = rospy.Publisher('calibration_failed', Trigger, queue_size=1)
        self.calibration_complete_pub = rospy.Publisher('calibration_complete', Trigger, queue_size=1)
        self.cycle_finished_pub = rospy.Publisher('cycle_finished', Trigger, queue_size=1)

        self.rate = rospy.Rate(self.control_frequency)

        rospy.loginfo("Single Motor Control Node initialized. Waiting for calibration...")

    def load_configuration(self):
        """Load configuration for a single motor."""
        try:
            self.torque_threshold = rospy.get_param('~torque_threshold', 8.0)
            self.calibration_speed = math.radians(rospy.get_param('~calibration_speed_deg', 30.0))
            self.max_calibration_time = rospy.get_param('~max_calibration_time', 30.0)

            self.gains = {
                'calibration': {'kp': rospy.get_param('~gains/calibration/kp', 10.0), 'kd': rospy.get_param('~gains/calibration/kd', 1.0)},
                'trajectory': {'kp': rospy.get_param('~gains/trajectory/kp', 50.0), 'kd': rospy.get_param('~gains/trajectory/kd', 2.0)}
            }

            self.motor_config_params = {
                'motor_id': rospy.get_param('~motor/id', 1),
                'controller_id': rospy.get_param('~motor/controller_id', 0x06),
                'joint_name': rospy.get_param('~motor/joint_name', "test_motor"),
                'motor_model': rospy.get_param('~motor/model', "AK80_64"),
                'angle_limits_deg': rospy.get_param('~motor/angle_limits_deg', [-45, 45]),
                'motor_direction': rospy.get_param('~motor/direction', 1)
            }

            # CAN interface parameters
            self.can_bustype = rospy.get_param('~can_bustype', 'socketcan')
            self.can_channel = rospy.get_param('~can_channel', 'can0')
            self.can_bitrate = rospy.get_param('~can_bitrate', 1000000)

            rospy.loginfo("Configuration loaded.")
        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            rospy.signal_shutdown("Configuration error")

    def setup_motor_configuration(self):
        """Setup the single motor configuration."""
        try:
            model_map = {m.value: m for m in motor_driver.MotorModel}
            model_str = self.motor_config_params['motor_model']
            if model_str not in model_map:
                rospy.logerr(f"Unknown motor model: {model_str}")
                rospy.signal_shutdown("Configuration error")
                return

            self.motor_config = MotorConfig(
                motor_id=self.motor_config_params['motor_id'],
                model=model_map[model_str],
                controller_id=self.motor_config_params['controller_id'],
                joint_name=self.motor_config_params['joint_name'],
                min_angle_deg=self.motor_config_params['angle_limits_deg'][0],
                max_angle_deg=self.motor_config_params['angle_limits_deg'][1]
            )
            self.motor_config.direction = self.motor_config_params['motor_direction']
            rospy.loginfo("Motor configuration setup complete.")
        except Exception as e:
            rospy.logerr(f"Error setting up motor configuration: {e}")
            rospy.signal_shutdown("Configuration error")

    def initialize_can_interface(self):
        """Initialize CAN interface."""
        try:
            rospy.loginfo(f"Initializing CAN bus: bustype={self.can_bustype}, channel={self.can_channel}, bitrate={self.can_bitrate}")
            self.can_channel = can.interface.Bus(
                channel=self.can_channel, 
                bustype=self.can_bustype, 
                bitrate=self.can_bitrate
            )
            rospy.loginfo("CAN interface initialized.")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize CAN interface: {e}")
            return False

    def initialize_motor(self):
        """Initialize the single motor controller."""
        try:
            self.motor_controller = motor_driver.MotorController(
                model=self.motor_config.model,
                controller_id=self.motor_config.controller_id
            )
            self.motor_state = motor_driver.MotorState()
            rospy.loginfo(f"Motor {self.motor_config.joint_name} initialized.")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize motor: {e}")
            return False

    def generate_trajectory(self):
        """Generates a simple trajectory ."""
        min_angle = self.motor_config.min_limit #math.radians(-30)
        max_angle = self.motor_config.max_limit #math.radians(30)
        # Create a path from min to max and back to min
        forward_path = np.linspace(min_angle, max_angle, 150)
        backward_path = np.linspace(max_angle, min_angle, 150)
        self.trajectory = np.concatenate((forward_path, backward_path))
        rospy.loginfo(f"Generated trajectory with {len(self.trajectory)} points.")

    def e_stop_callback(self, msg):
        if msg.trigger:
            with self.state_lock:
                self.is_emergency_stop = True
                self.system_state = "E_STOP"
            rospy.logwarn("Emergency stop triggered - initiating shutdown sequence")
            
            # Stop the motor immediately
            self.emergency_stop_motor()
            
            # Update calibration state if needed
            if self.calibration_state == CalibrationState.START_CALIBRATION:
                self.calibration_state = CalibrationState.FAILED
            
            # Perform clean shutdown
            self.perform_emergency_shutdown()
            
            # Signal ROS to shutdown this node
            rospy.signal_shutdown("Emergency stop triggered")
        else:
            self.is_emergency_stop = False
            # Update system state from message if available
            if hasattr(msg, 'state'):
                self.handle_state_transition(msg.state)




    def system_state_callback(self, msg):
        """Handle system state updates from emergency stop node."""
        # For now, we'll monitor emergency stop node's published states via logs
        # and track state through the emergency stop trigger message
        rospy.loginfo("System state update received from emergency stop node")


    def perform_calibration(self):
        """Complete calibration process including motor setup and limit detection"""
        rospy.loginfo(f"Starting calibration for motor {self.motor_config.joint_name}")
        
        try:
            self.motor_config.is_calibrated = False
            
            # Step 1: Flush buffer before starting calibration
            rospy.loginfo("  Flushing CAN buffer...")
            motor_driver.flush_can_buffer(self.can_channel, 0.2)
            
            # Step 2: Enter MIT mode
            rospy.loginfo("  Entering MIT mode...")
            if not motor_driver.enter_mode(self.can_channel, self.motor_controller.controller_id):
                rospy.logerr("Failed to enter MIT mode")
                return False
            time.sleep(0.1)
            
            # Read response after entering mode
            motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                         max_attempts=3, timeout_ms=10, debug_flag=self.debug_flag)
            rospy.loginfo("  Motor entered MIT mode")
            
            # Step 3: Safe zero position (this takes up to 2 seconds and blocks until complete)
            rospy.loginfo("  Setting safe zero position (this will take ~2 seconds)...")
            if not motor_driver.safe_zero_position(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag):
                rospy.logerr("Failed to safely zero position")
                return False
            rospy.loginfo("  Zero position set successfully")
            
            # Step 4: Perform limit detection
            rospy.loginfo("  Starting limit detection...")
            limits = []
            for direction in [1, -1]:
                rospy.loginfo(f"    Searching for limit in direction {direction}")
                self.motor_state.p_in = 0.0  # Position is not used, so set kp=0
                self.motor_state.v_in = direction * self.calibration_speed * self.motor_config.direction
                self.motor_state.kp_in = 0.0  # Zero kp when position is not used
                self.motor_state.kd_in = self.gains['calibration']['kd']
                self.motor_state.t_in = 0.0
                
                start_time = time.time()
                while time.time() - start_time < self.max_calibration_time:
                    if self.is_emergency_stop: 
                        rospy.logerr("Calibration aborted due to emergency stop")
                        return False
                    
                    # Send command
                    motor_driver.pack_cmd(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag)
                    time.sleep(0.01)
                    
                    # Always read response after sending command
                    if motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                                    max_attempts=3, timeout_ms=5, debug_flag=self.debug_flag):
                        # Check if motor position exceeds configured angle limits in the direction of movement
                        limit_exceeded = False
                        if direction > 0 and self.motor_state.p_out > self.motor_config.max_angle_rad:
                            rospy.logwarn(f"    Motor position {self.motor_state.p_out:.3f} rad exceeds max limit "
                                         f"{self.motor_config.max_angle_rad:.3f} rad in positive direction")
                            limit_exceeded = True
                        elif direction < 0 and self.motor_state.p_out < self.motor_config.min_angle_rad:
                            rospy.logwarn(f"    Motor position {self.motor_state.p_out:.3f} rad exceeds min limit "
                                         f"{self.motor_config.min_angle_rad:.3f} rad in negative direction")
                            limit_exceeded = True
                        
                        if limit_exceeded:
                            # Add the configured angle limit as a valid calibration limit
                            if direction > 0:
                                limits.append(self.motor_config.max_angle_rad)
                                rospy.loginfo(f"    Configured max angle limit reached at {self.motor_config.max_angle_rad:.3f} rad")
                            else:
                                limits.append(self.motor_config.min_angle_rad)
                                rospy.loginfo(f"    Configured min angle limit reached at {self.motor_config.min_angle_rad:.3f} rad")
                            
                            # Stop motion immediately when angle limit is reached
                            self.motor_state.v_in = 0.0
                            self.motor_state.kp_in = 0.0  # Zero kp when position is not used
                            motor_driver.pack_cmd(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag)
                            time.sleep(0.1)
                            # Read response after stopping
                            motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                                          max_attempts=3, timeout_ms=5, debug_flag=self.debug_flag)
                            time.sleep(0.2)
                            break
                        
                        if abs(self.motor_state.t_out) > self.torque_threshold:
                            limits.append(self.motor_state.p_out)
                            rospy.loginfo(f"    Limit found at {self.motor_state.p_out:.3f} rad")
                            
                            # Stop motion when limit is found
                            self.motor_state.v_in = 0.0
                            self.motor_state.kp_in = 0.0  # Zero kp when position is not used
                            motor_driver.pack_cmd(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag)
                            time.sleep(0.1)
                            # Read response after stopping
                            motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                                          max_attempts=3, timeout_ms=5, debug_flag=self.debug_flag)
                            time.sleep(0.2)
                            break

                else:
                    rospy.logerr("Calibration timeout - limit not found")
                    self.trigger_emergency_stop_and_shutdown("Calibration timeout - limit not found")
                    return False

            if len(limits) != 2:
                rospy.logerr(f"Expected 2 limits, found {len(limits)}")
                return False
            
            # Step 5: Set calibration results
            self.motor_config.min_limit = min(limits)
            self.motor_config.max_limit = max(limits)
            self.motor_config.is_calibrated = True

            self.generate_trajectory()
            
            rospy.loginfo(f"  Limits detected: min={self.motor_config.min_limit:.3f}, max={self.motor_config.max_limit:.3f}")
            
            # Step 6: Move to center position
            center_position = self.motor_config.min_limit #(self.motor_config.min_limit + self.motor_config.max_limit) / 2.0
            rospy.loginfo(f"  Moving to center position: {center_position:.3f} rad")
            
            self.motor_state.p_in = center_position
            self.motor_state.v_in = 0.0
            self.motor_state.kp_in = self.gains['calibration']['kp']  # Use kp when position is used
            motor_driver.pack_cmd(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag)
            time.sleep(0.1)
            # Read response after moving to center
            motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                         max_attempts=3, timeout_ms=5, debug_flag=self.debug_flag)
            time.sleep(2.0)
            
            rospy.loginfo("Calibration completed successfully!")
            # Send calibration complete message
            self.send_calibration_complete()
            return True
            
        except Exception as e:
            rospy.logerr(f"Error during calibration: {e}")
            # Send calibration failed message
            self.send_calibration_failed()
            return False

    def reset_calibration(self):
        self.calibration_state = CalibrationState.NOT_STARTED
        self.motor_config.is_calibrated = False
        
        # Flush buffer before exiting
        motor_driver.flush_can_buffer(self.can_channel, 0.2)
        
        motor_driver.exit_mode(self.can_channel, self.motor_controller.controller_id)
        time.sleep(0.1)
        # Read response after exiting mode
        motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                     max_attempts=3, timeout_ms=50, debug_flag=self.debug_flag)
        rospy.loginfo("Calibration reset.")

    def emergency_stop_motor(self):
        with self.motor_lock:
            self.motor_state.p_in = self.motor_state.p_out
            self.motor_state.v_in = 0.0
            self.motor_state.kp_in = 0.0
            self.motor_state.kd_in = 5.0
            self.motor_state.t_in = 0.0
            motor_driver.pack_cmd(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag)
            time.sleep(0.01)
            # Read response after emergency stop command
            motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                         max_attempts=1, timeout_ms=50, debug_flag=self.debug_flag)
        rospy.logwarn("Emergency stop triggered for motor.")

    def perform_emergency_shutdown(self):
        """Perform complete emergency shutdown sequence"""
        rospy.logwarn("Performing emergency shutdown sequence...")
        
        try:
            if self.can_channel and self.motor_controller:
                # Flush CAN buffer before exiting
                rospy.loginfo("Flushing CAN buffer...")
                motor_driver.flush_can_buffer(self.can_channel, 0.2)
                
                # Exit MIT mode
                rospy.loginfo("Exiting MIT mode...")
                motor_driver.exit_mode(self.can_channel, self.motor_controller.controller_id)
                time.sleep(0.1)
                
                # Read final status
                motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                             max_attempts=1, timeout_ms=50, debug_flag=self.debug_flag)
                
                # Close CAN channel
                rospy.loginfo("Closing CAN channel...")
                self.can_channel.shutdown()
                self.can_channel = None
                
            rospy.logwarn("Emergency shutdown sequence completed")
            
        except Exception as e:
            rospy.logerr(f"Error during emergency shutdown: {e}")

    def update_motor_state(self):
        with self.motor_lock:
            # if motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
            #                                 max_attempts=2, timeout_ms=100, debug_flag=self.debug_flag):
            self.motor_position = self.motor_state.p_out
            self.motor_velocity = self.motor_state.v_out
            self.motor_torque = self.motor_state.t_out
            self.motor_temperature = self.motor_state.temperature
            self.motor_error_flag = self.motor_state.error_flag
            # else:
            #     rospy.logwarn_throttle(1.0, "Failed to Update motor state")

    def send_motor_command(self):
        if not self.motor_config.is_calibrated:
            return False
        if self.is_emergency_stop:
            return False
        
        # Only send commands during WALKING state
        with self.state_lock:
            if self.system_state != "WALKING":
                return False

        with self.motor_lock:
            self.desired_position = self.trajectory[self.trajectory_index]
            self.trajectory_index = (self.trajectory_index + 1) % len(self.trajectory)

            self.motor_state.p_in = np.clip(self.desired_position, self.motor_config.min_limit, self.motor_config.max_limit)
            self.motor_state.v_in = 0.0  # Zero velocity
            self.motor_state.kp_in = self.gains['trajectory']['kp']
            self.motor_state.kd_in = self.gains['trajectory']['kd']
            self.motor_state.t_in = 0.0  # Zero feedforward torque
            
            # Send command
            motor_driver.pack_cmd(self.can_channel, self.motor_controller, self.motor_state, debug_flag=self.debug_flag)
            time.sleep(0.001)  # Brief delay for motor response
            
            # Always read response after sending command to clear buffer
            motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                         max_attempts=3, timeout_ms=1, debug_flag=self.debug_flag)

    def publish_motor_status(self):
        msg = MotorStatus()
        msg.header.stamp = rospy.Time.now()
        msg.motor_ids = [self.motor_config.motor_id]
        msg.joint_names = [self.motor_config.joint_name]
        msg.calibrated_flags = [self.motor_config.is_calibrated]
        # Add calibration failed field (True if calibration failed)
        calibration_failed = (self.calibration_state == CalibrationState.FAILED)
        if hasattr(msg, 'calibration_failed'):
            msg.calibration_failed = [calibration_failed]
        msg.positions = [self.motor_position]
        msg.velocities = [self.motor_velocity]
        msg.torques = [self.motor_torque]
        msg.temperatures = [self.motor_temperature]
        msg.error_flags = [self.motor_error_flag]
        self.motor_status_pub.publish(msg)

    def handle_state_transition(self, new_state):
        """Handle state transitions and perform appropriate actions."""
        with self.state_lock:
            if self.previous_state == new_state:
                return  # No state change
                
            rospy.loginfo(f"State transition: {self.previous_state} -> {new_state}")
            
            # Handle transitions based on current state machine logic
            if new_state == "CALIBRATION_PROCESS" and self.previous_state == "INIT":
                # Start calibration when entering CALIBRATION_PROCESS from INIT
                if self.calibration_state == CalibrationState.NOT_STARTED:
                    self.calibration_state = CalibrationState.START_CALIBRATION
                    rospy.loginfo("Calibration will start on next cycle")
                    
            elif new_state == "STOPPING" and self.previous_state == "WALKING":
                # When entering STOPPING from WALKING, signal cycle finished immediately for single motor
                rospy.loginfo("Entering STOPPING state - single motor will signal cycle finished")
                # For single motor, we can finish the cycle immediately
                rospy.Timer(rospy.Duration(1.0), lambda event: self.send_cycle_finished(), oneshot=True)
                
            self.previous_state = self.system_state
            self.system_state = new_state
            
    def run(self):
        rospy.loginfo("Starting single motor control loop...")
        while not rospy.is_shutdown():
            # Exit immediately if emergency stop is triggered
            if self.is_emergency_stop:
                rospy.loginfo("Emergency stop active - exiting control loop")
                break
            
            # Get current system state (thread-safe)
            with self.state_lock:
                current_state = self.system_state
                
            # State-based logic following the emergency stop node state machine
            if current_state == "INIT":
                # In INIT state - wait for emergency stop node to transition to CALIBRATION_PROCESS
                rospy.loginfo_throttle(5, "Single motor in INIT state - waiting for calibration trigger from emergency stop")
                
            elif current_state == "CALIBRATION_PROCESS":
                # Handle calibration when in CALIBRATION_PROCESS state
                if self.calibration_state == CalibrationState.START_CALIBRATION:
                    rospy.loginfo("Starting calibration process...")
                    if self.perform_calibration():
                        self.calibration_state = CalibrationState.COMPLETED
                        rospy.loginfo("Calibration completed - ready for trajectory execution")
                    else:
                        self.calibration_state = CalibrationState.FAILED
                        rospy.logerr("Calibration failed")
                        # Send calibration failed message
                        self.send_calibration_failed()
                        self.reset_calibration()
                elif self.calibration_state == CalibrationState.NOT_STARTED:
                    # Trigger calibration when entering this state
                    self.calibration_state = CalibrationState.START_CALIBRATION
                    
            elif current_state == "READY":
                # In READY state - motors calibrated, waiting for walking command
                if self.calibration_state == CalibrationState.COMPLETED:
                    rospy.loginfo_throttle(10, "Single motor READY - waiting for walking command from emergency stop")
                    self.update_motor_state()  # Keep updating motor state
                else:
                    rospy.logwarn_throttle(5, "In READY state but calibration not completed")
                    
            elif current_state == "WALKING":
                # In WALKING state - execute trajectory
                if self.calibration_state == CalibrationState.COMPLETED:
                    self.send_motor_command()
                    self.update_motor_state()
                else:
                    rospy.logerr("Cannot walk - calibration not completed")
                    
            elif current_state == "STOPPING":
                # In STOPPING state - finish current cycle and return to ready
                rospy.loginfo_throttle(2, "Single motor in STOPPING state - cycle will finish soon")
                self.update_motor_state()
                # Cycle finished signal is sent in handle_state_transition
                
            elif current_state == "E_STOP":
                # Emergency stop state - should not reach here as node will shutdown
                rospy.logerr("Single motor in E_STOP state - shutting down")
                break
                
            else:
                rospy.logwarn_throttle(5, f"Unknown system state: {current_state}")
            
            # Always publish status (unless emergency stopped)
            if not self.is_emergency_stop:
                self.publish_motor_status()
                
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down single motor control node...")
        
        # Only perform shutdown if emergency shutdown hasn't already been done
        if not self.is_emergency_stop:
            self.emergency_stop_motor()
            if self.can_channel and self.motor_controller:
                # Flush buffer before exiting
                motor_driver.flush_can_buffer(self.can_channel, 0.2)
                motor_driver.exit_mode(self.can_channel, self.motor_controller.controller_id)
                time.sleep(0.1)
                # Read response after exiting mode
                motor_driver.read_motor_status(self.can_channel, self.motor_controller, self.motor_state, 
                                             max_attempts=1, timeout_ms=50, debug_flag=self.debug_flag)
            if self.can_channel:
                self.can_channel.shutdown()
        else:
            rospy.loginfo("Emergency shutdown already performed - skipping redundant shutdown")

    def send_calibration_failed(self):
        """Send calibration failed message to emergency stop node."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.calibration_failed_pub.publish(msg)
        rospy.logwarn("Calibration failed message sent")

    def send_calibration_complete(self):
        """Send calibration complete message to emergency stop node."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.calibration_complete_pub.publish(msg)
        rospy.loginfo("Calibration complete message sent")
        
    def send_cycle_finished(self):
        """Send cycle finished message when stopping is complete."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.cycle_finished_pub.publish(msg)
        rospy.loginfo("Cycle finished message sent")

    def trigger_emergency_stop_and_shutdown(self, reason="Single motor control emergency"):
        """Trigger emergency stop and shutdown the node."""
        rospy.logerr(f"SINGLE MOTOR CONTROL EMERGENCY: {reason}")
        
        # Set emergency stop flag
        self.is_emergency_stop = True
        
        # Stop motor immediately
        self.emergency_stop_motor()
        
        # Send emergency stop message
        e_stop_msg = EStopTrigger()
        e_stop_msg.header.stamp = rospy.Time.now()
        e_stop_msg.trigger = True
        e_stop_msg.state = "SINGLE_MOTOR_EMERGENCY"
        self.e_stop_trigger_pub.publish(e_stop_msg)
        
        # Shutdown after brief delay
        rospy.Timer(rospy.Duration(0.5), lambda event: rospy.signal_shutdown(reason), oneshot=True)

if __name__ == '__main__':
    try:
        node = SingleMotorControlNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass