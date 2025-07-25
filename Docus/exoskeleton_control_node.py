#!/usr/bin/env python

import rospy
import threading
import time
from enum import Enum
from std_msgs.msg import Bool, Float64MultiArray, Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from exoskeleton_msgs.msg import ExoskeletonState, TrajectoryCommand, MotorStatus
from exoskeleton_msgs.srv import DynamicModelFF, DynamicModelFFRequest, DynamicModelFFResponse
import candle_driver

# Import the motor controller from our previous implementation
from motor_controller import MotorController, MotorModel, MotorState
from motor_controller import enter_mode, exit_mode, pack_cmd, read_motor_status

class JointID(Enum):
    RIGHT_HIP = 0
    RIGHT_KNEE = 1
    LEFT_HIP = 2
    LEFT_KNEE = 3

class ExoskeletonMotorController:
    def __init__(self):
        rospy.init_node('exoskeleton_motor_controller', anonymous=False)
        
        # Node parameters - Conservative default for reliability
        self.rate = rospy.get_param('~control_rate', 50)  # Hz - Conservative default
        self.can_device_index = rospy.get_param('~can_device_index', 0)
        
        # Motor models per joint type
        self.motor_models = {
            JointID.RIGHT_HIP: MotorModel.AK80_64,
            JointID.RIGHT_KNEE: MotorModel.AK80_8,
            JointID.LEFT_HIP: MotorModel.AK80_64,
            JointID.LEFT_KNEE: MotorModel.AK80_8
        }
        
        # Motor controller IDs (CAN IDs)
        self.motor_ids = {
            JointID.RIGHT_HIP: rospy.get_param('~right_hip_id', 0x01),
            JointID.RIGHT_KNEE: rospy.get_param('~right_knee_id', 0x02),
            JointID.LEFT_HIP: rospy.get_param('~left_hip_id', 0x03),
            JointID.LEFT_KNEE: rospy.get_param('~left_knee_id', 0x04)
        }
        
        # Control parameters
        self.default_kp = rospy.get_param('~default_kp', 50.0)
        self.default_kd = rospy.get_param('~default_kd', 2.0)
        
        # Calibration parameters
        self.calibration_torque_threshold = rospy.get_param('~calibration_torque_threshold', 8.0)  # Nm
        self.calibration_velocity = rospy.get_param('~calibration_velocity', 0.2)  # rad/s
        self.calibration_timeout = rospy.get_param('~calibration_timeout', 30.0)  # seconds per joint
        
        # Safety parameters
        self.emergency_stop = False
        self.motors_enabled = False
        self.calibration_mode = False
        self.calibration_status = {joint: False for joint in JointID}  # Track individual joint calibration
        self.joint_limits = {}  # Will store calibrated limits for each joint
        self.system_calibrated = False
        
        # Initialize motor controllers
        self.motor_controllers = {}
        self.motor_states = {}
        self.initialize_motors()
        
        # Initialize CAN communication
        self.can_channel = None
        self.initialize_can()
        
        # Current robot state
        self.current_state = ExoskeletonState()
        self.current_state.header = Header()
        self.current_state.joint_names = ['right_hip', 'right_knee', 'left_hip', 'left_knee']
        self.current_state.position = [0.0] * 4
        self.current_state.velocity = [0.0] * 4
        self.current_state.torque = [0.0] * 4
        self.current_state.temperature = [0.0] * 4
        self.current_state.emergency_stop = False
        self.current_state.calibrated = False
        
        # Desired trajectory
        self.desired_trajectory = TrajectoryCommand()
        self.trajectory_received = False
        
        # Thread locks
        self.state_lock = threading.Lock()
        self.trajectory_lock = threading.Lock()
        self.emergency_lock = threading.Lock()
        
        # Initialize ROS communication
        self.initialize_ros_communication()
        
        rospy.loginfo("Exoskeleton Motor Controller initialized")
        
    def initialize_motors(self):
        """Initialize motor controllers for each joint with specific models"""
        for joint_id in JointID:
            controller_id = self.motor_ids[joint_id]
            motor_model = self.motor_models[joint_id]
            self.motor_controllers[joint_id] = MotorController(motor_model, controller_id)
            self.motor_states[joint_id] = MotorState()
            
        rospy.loginfo("Initialized motor controllers:")
        for joint_id in JointID:
            model_name = self.motor_models[joint_id].value
            rospy.loginfo(f"  {joint_id.name}: {model_name} (ID: 0x{self.motor_ids[joint_id]:02X})")
        
    def initialize_can(self):
        """Initialize CAN communication"""
        try:
            devices = candle_driver.list_devices()
            if not devices:
                rospy.logerr("No CAN devices found")
                return False
                
            device = devices[self.can_device_index]
            rospy.loginfo(f"Using CAN device: {device.name()}")
            
            if not device.open():
                rospy.logerr("Failed to open CAN device")
                return False
                
            self.can_channel = device.channel(0)
            self.can_channel.set_bitrate(1000000)  # 1 Mbps
            
            if not self.can_channel.start():
                rospy.logerr("Failed to start CAN channel")
                device.close()
                return False
                
            rospy.loginfo("CAN communication initialized successfully")
            return True
            
        except Exception as e:
            rospy.logerr(f"CAN initialization error: {e}")
            return False
            
    def initialize_ros_communication(self):
        """Initialize ROS publishers, subscribers, and services"""
        
        # Publishers
        self.state_publisher = rospy.Publisher(
            '/exoskeleton/state', 
            ExoskeletonState, 
            queue_size=1
        )
        
        self.joint_state_publisher = rospy.Publisher(
            '/joint_states', 
            JointState, 
            queue_size=1
        )
        
        # Subscribers
        self.trajectory_subscriber = rospy.Subscriber(
            '/exoskeleton/trajectory_command',
            TrajectoryCommand,
            self.trajectory_callback,
            queue_size=1
        )
        
        self.emergency_subscriber = rospy.Subscriber(
            '/exoskeleton/emergency_stop',
            Bool,
            self.emergency_stop_callback,
            queue_size=1
        )
        
        self.calibration_subscriber = rospy.Subscriber(
            '/exoskeleton/calibrate',
            Bool,
            self.calibration_callback,
            queue_size=1
        )
        
        # Service clients
        self.dynamic_model_client = rospy.ServiceProxy(
            '/exoskeleton/dynamic_model_feedforward',
            DynamicModelFF
        )
        
        # Wait for service
        rospy.loginfo("Waiting for dynamic model service...")
        try:
            rospy.wait_for_service('/exoskeleton/dynamic_model_feedforward', timeout=10)
            rospy.loginfo("Dynamic model service available")
        except rospy.ROSException:
            rospy.logwarn("Dynamic model service not available - continuing without feedforward")
            
    def trajectory_callback(self, msg):
        """Callback for trajectory commands from trajectory generator"""
        with self.trajectory_lock:
            if not self.emergency_stop and not self.calibration_mode:
                self.desired_trajectory = msg
                self.trajectory_received = True
                
    def emergency_stop_callback(self, msg):
        """Callback for emergency stop commands"""
        with self.emergency_lock:
            if msg.data and not self.emergency_stop:
                rospy.logwarn("EMERGENCY STOP ACTIVATED")
                self.emergency_stop = True
                self.execute_emergency_stop()
            elif not msg.data and self.emergency_stop:
                rospy.loginfo("Emergency stop released - manual restart required")
                
    def calibration_callback(self, msg):
        """Callback for calibration commands"""
        if msg.data and not self.calibration_mode and not self.emergency_stop:
            rospy.loginfo("Starting calibration routine")
            self.calibration_mode = True
            # TODO: Implement calibration routine in separate thread
            threading.Thread(target=self.calibration_routine).start()
            
    def execute_emergency_stop(self):
        """Execute emergency stop procedure"""
        try:
            rospy.logwarn("Executing emergency stop - disabling all motors")
            
            # Exit MIT mode for all motors
            for joint_id in JointID:
                controller_id = self.motor_controllers[joint_id].controller_id
                exit_mode(self.can_channel, controller_id)
                time.sleep(0.01)  # Brief delay between commands
                
            self.motors_enabled = False
            rospy.logwarn("All motors disabled")
            
        except Exception as e:
            rospy.logerr(f"Error during emergency stop: {e}")
            
    def enable_motors(self):
        """Enable all motors and enter MIT mode"""
        if self.emergency_stop:
            rospy.logwarn("Cannot enable motors - emergency stop active")
            return False
            
        try:
            rospy.loginfo("Enabling motors - entering MIT mode")
            
            for joint_id in JointID:
                controller_id = self.motor_controllers[joint_id].controller_id
                if not enter_mode(self.can_channel, controller_id):
                    rospy.logerr(f"Failed to enable motor {joint_id.name}")
                    return False
                time.sleep(0.01)  # Brief delay between commands
                
            self.motors_enabled = True
            rospy.loginfo("All motors enabled")
            return True
            
        except Exception as e:
            rospy.logerr(f"Error enabling motors: {e}")
            return False
            
    def calculate_feedforward_torque(self):
        """Calculate feedforward torque using dynamic model service"""
        try:
            # Prepare service request
            request = DynamicModelFFRequest()
            request.current_state = self.current_state
            request.desired_trajectory = self.desired_trajectory
            
            # Call service
            response = self.dynamic_model_client(request)
            
            if response.success:
                return response.feedforward_torque
            else:
                rospy.logwarn(f"Dynamic model service failed: {response.message}")
                return [0.0] * 4
                
        except rospy.ServiceException as e:
            rospy.logwarn(f"Dynamic model service call failed: {e}")
            return [0.0] * 4
            
    def read_all_motor_states(self):
        """Read current state from all motors synchronously"""
        success_count = 0
        
        for joint_id in JointID:
            motor_controller = self.motor_controllers[joint_id]
            motor_state = self.motor_states[joint_id]
            
            if read_motor_status(self.can_channel, motor_controller, motor_state, max_attempts=2, timeout_ms=50):
                success_count += 1
            else:
                rospy.logwarn(f"Failed to read status from {joint_id.name}")
                
        return success_count == len(JointID)
        
    def send_control_commands(self):
        """Send control commands to all motors synchronously with timing optimization"""
        if not self.motors_enabled or self.emergency_stop:
            return False
            
        # Get feedforward torques (with timeout to prevent blocking)
        feedforward_torques = [0.0] * 4
        if self.trajectory_received:
            try:
                # Use shorter timeout for service call to maintain real-time performance
                rospy.wait_for_service('/exoskeleton/dynamic_model_feedforward', timeout=0.001)
                feedforward_torques = self.calculate_feedforward_torque()
            except rospy.ROSException:
                # Continue without feedforward if service is slow
                rospy.logwarn_throttle(5.0, "Dynamic model service timeout - continuing without feedforward")
                
        success_count = 0
        start_time = time.time()
        
        with self.trajectory_lock:
            if not self.trajectory_received:
                return True  # No commands to send
                
            desired_pos = self.desired_trajectory.position
            desired_vel = self.desired_trajectory.velocity
            kp_gains = self.desired_trajectory.kp if len(self.desired_trajectory.kp) == 4 else [self.default_kp] * 4
            kd_gains = self.desired_trajectory.kd if len(self.desired_trajectory.kd) == 4 else [self.default_kd] * 4
            
        # Send commands to all motors with minimal delay between sends
        for i, joint_id in enumerate(JointID):
            motor_controller = self.motor_controllers[joint_id]
            motor_state = self.motor_states[joint_id]
            
            # Set control parameters
            motor_state.p_in = desired_pos[i]
            motor_state.v_in = desired_vel[i]
            motor_state.kp_in = kp_gains[i]
            motor_state.kd_in = kd_gains[i]
            motor_state.t_in = feedforward_torques[i]
            
            # Send command
            if pack_cmd(self.can_channel, motor_controller, motor_state):
                success_count += 1
            else:
                rospy.logerr(f"Failed to send command to {joint_id.name}")
                
        # Log timing for performance monitoring
        send_time_ms = (time.time() - start_time) * 1000
        if send_time_ms > 5.0:  # Warn if sending takes too long
            rospy.logwarn(f"Command sending took {send_time_ms:.1f}ms (target: <5ms)")
                
        return success_count == len(JointID)
        
    def update_robot_state(self):
        """Update robot state from motor feedback"""
        with self.state_lock:
            self.current_state.header.stamp = rospy.Time.now()
            
            for i, joint_id in enumerate(JointID):
                motor_state = self.motor_states[joint_id]
                self.current_state.position[i] = motor_state.p_out
                self.current_state.velocity[i] = motor_state.v_out
                self.current_state.torque[i] = motor_state.t_out
                self.current_state.temperature[i] = getattr(motor_state, 'temperature', 0.0)
            
            # Update system status
            self.current_state.emergency_stop = self.emergency_stop
            self.current_state.calibrated = self.system_calibrated
                
    def publish_robot_state(self):
        """Publish current robot state"""
        with self.state_lock:
            # Publish custom exoskeleton state
            self.state_publisher.publish(self.current_state)
            
            # Publish standard joint states
            joint_state = JointState()
            joint_state.header = self.current_state.header
            joint_state.name = self.current_state.joint_names
            joint_state.position = self.current_state.position
            joint_state.velocity = self.current_state.velocity
            joint_state.effort = self.current_state.torque
            
            self.joint_state_publisher.publish(joint_state)
            
    def calibration_routine(self):
        """
        Calibration routine to find joint limits by detecting mechanical stoppers
        
        For each joint:
        1. Move slowly in one direction until torque threshold is reached (first limit)
        2. Move slowly in opposite direction until torque threshold is reached (second limit)
        3. Calculate center position and set as zero
        4. Store joint limits for safety
        """
        rospy.loginfo("=== Starting Calibration Routine ===")
        
        try:
            if not self.motors_enabled:
                if not self.enable_motors():
                    rospy.logerr("Failed to enable motors for calibration")
                    return
            
            # Reset calibration status
            self.calibration_status = {joint: False for joint in JointID}
            self.joint_limits = {}
            
            # Calibrate each joint individually
            for joint_id in JointID:
                rospy.loginfo(f"\nCalibrating {joint_id.name}...")
                
                if self.calibrate_single_joint(joint_id):
                    self.calibration_status[joint_id] = True
                    rospy.loginfo(f"✓ {joint_id.name} calibration successful")
                else:
                    rospy.logerr(f"✗ {joint_id.name} calibration failed")
                    self.calibration_mode = False
                    return
                    
                # Brief pause between joint calibrations
                time.sleep(1.0)
            
            # Check if all joints are calibrated
            if all(self.calibration_status.values()):
                self.system_calibrated = True
                rospy.loginfo("=== System Calibration Complete ===")
                rospy.loginfo("Joint limits found:")
                for joint_id, limits in self.joint_limits.items():
                    rospy.loginfo(f"  {joint_id.name}: {limits['min']:.3f} to {limits['max']:.3f} rad")
            else:
                rospy.logerr("Calibration incomplete - some joints failed")
                
        except Exception as e:
            rospy.logerr(f"Calibration routine failed: {e}")
        finally:
            self.calibration_mode = False
            
    def calibrate_single_joint(self, joint_id: JointID):
        """
        Calibrate a single joint by finding both limits
        
        Args:
            joint_id: Joint identifier to calibrate
            
        Returns:
            bool: True if calibration successful, False otherwise
        """
        motor_controller = self.motor_controllers[joint_id]
        motor_state = self.motor_states[joint_id]
        
        # Get current position as starting point
        if not read_motor_status(self.can_channel, motor_controller, motor_state):
            rospy.logerr(f"Failed to read initial position for {joint_id.name}")
            return False
            
        start_position = motor_state.p_out
        rospy.loginfo(f"Starting calibration from position: {start_position:.3f} rad")
        
        limits = {}
        
        # Find both limits
        for direction in [1, -1]:  # Positive then negative direction
            direction_name = "positive" if direction > 0 else "negative"
            rospy.loginfo(f"Finding {direction_name} limit...")
            
            limit_position = self.find_joint_limit(joint_id, direction, start_position)
            if limit_position is None:
                rospy.logerr(f"Failed to find {direction_name} limit for {joint_id.name}")
                return False
                
            limits[direction] = limit_position
            rospy.loginfo(f"Found {direction_name} limit at {limit_position:.3f} rad")
            
            # Return to starting position before finding other limit
            if not self.move_to_position_slowly(joint_id, start_position):
                rospy.logerr(f"Failed to return to start position for {joint_id.name}")
                return False
                
            time.sleep(0.5)  # Brief pause
        
        # Calculate joint limits and center position
        min_limit = min(limits[1], limits[-1])
        max_limit = max(limits[1], limits[-1])
        center_position = (min_limit + max_limit) / 2.0
        
        # Store joint limits
        self.joint_limits[joint_id] = {
            'min': min_limit,
            'max': max_limit,
            'range': max_limit - min_limit,
            'center': center_position
        }
        
        # Move to center position
        rospy.loginfo(f"Moving to center position: {center_position:.3f} rad")
        if not self.move_to_position_slowly(joint_id, center_position):
            rospy.logerr(f"Failed to move to center position for {joint_id.name}")
            return False
            
        # Set zero position at center
        rospy.loginfo("Setting zero position at center...")
        if not zero_position(self.can_channel, motor_controller.controller_id):
            rospy.logerr(f"Failed to set zero position for {joint_id.name}")
            return False
            
        time.sleep(0.2)
        return True
        
    def find_joint_limit(self, joint_id: JointID, direction: int, start_position: float):
        """
        Find joint limit in specified direction by detecting torque increase
        
        Args:
            joint_id: Joint to calibrate
            direction: 1 for positive, -1 for negative direction
            start_position: Starting position in radians
            
        Returns:
            float: Position where limit was detected, or None if failed
        """
        motor_controller = self.motor_controllers[joint_id]
        motor_state = self.motor_states[joint_id]
        
        # Calibration control parameters
        velocity = self.calibration_velocity * direction
        kp_cal = 20.0  # Lower gain for gentle movement
        kd_cal = 1.0
        
        start_time = time.time()
        positions = []
        torques = []
        
        rospy.loginfo(f"Moving in direction {direction} at {abs(velocity):.2f} rad/s...")
        
        while time.time() - start_time < self.calibration_timeout:
            # Check for emergency stop
            if self.emergency_stop:
                rospy.logwarn("Emergency stop during calibration")
                return None
                
            # Set control command for slow movement
            motor_state.p_in = start_position  # Keep position target at start
            motor_state.v_in = velocity        # Use velocity control
            motor_state.kp_in = kp_cal
            motor_state.kd_in = kd_cal
            motor_state.t_in = 0.0
            
            # Send command
            if not pack_cmd(self.can_channel, motor_controller, motor_state):
                rospy.logerr("Failed to send calibration command")
                return None
                
            time.sleep(0.02)  # 50Hz update rate during calibration
            
            # Read motor status
            if not read_motor_status(self.can_channel, motor_controller, motor_state):
                rospy.logwarn("Failed to read motor status during calibration")
                continue
                
            current_position = motor_state.p_out
            current_torque = abs(motor_state.t_out)  # Use absolute torque
            
            positions.append(current_position)
            torques.append(current_torque)
            
            # Keep only recent measurements for trend analysis
            if len(positions) > 20:
                positions.pop(0)
                torques.pop(0)
                
            # Check for torque threshold (need consistent high torque)
            if len(torques) >= 5:
                recent_torques = torques[-5:]  # Last 5 measurements
                avg_torque = sum(recent_torques) / len(recent_torques)
                
                if avg_torque > self.calibration_torque_threshold:
                    rospy.loginfo(f"Limit detected! Torque: {avg_torque:.2f} Nm at position: {current_position:.3f} rad")
                    return current_position
                    
            # Safety check - don't move too far from start
            if abs(current_position - start_position) > 3.0:  # 3 radians max travel
                rospy.logwarn(f"Exceeded maximum travel distance without finding limit")
                return None
                
        rospy.logerr("Calibration timeout reached without finding limit")
        return None
        
    def move_to_position_slowly(self, joint_id: JointID, target_position: float, timeout: float = 10.0):
        """
        Move joint to target position slowly and safely
        
        Args:
            joint_id: Joint to move
            target_position: Target position in radians
            timeout: Maximum time to reach position
            
        Returns:
            bool: True if position reached successfully
        """
        motor_controller = self.motor_controllers[joint_id]
        motor_state = self.motor_states[joint_id]
        
        start_time = time.time()
        position_tolerance = 0.05  # 0.05 radians tolerance
        
        while time.time() - start_time < timeout:
            if self.emergency_stop:
                return False
                
            # Send position command
            motor_state.p_in = target_position
            motor_state.v_in = 0.0
            motor_state.kp_in = 30.0  # Moderate gain
            motor_state.kd_in = 2.0
            motor_state.t_in = 0.0
            
            if not pack_cmd(self.can_channel, motor_controller, motor_state):
                return False
                
            time.sleep(0.02)
            
            # Check position
            if read_motor_status(self.can_channel, motor_controller, motor_state):
                position_error = abs(motor_state.p_out - target_position)
                if position_error < position_tolerance:
                    return True
                    
        rospy.logwarn(f"Failed to reach target position {target_position:.3f} within timeout")
        return False
            
    def control_loop(self):
        """Main control loop with adaptive timing"""
        rate = rospy.Rate(self.rate)
        
        # Enable motors at startup
        if not self.enable_motors():
            rospy.logerr("Failed to enable motors - exiting")
            return
            
        rospy.loginfo(f"Starting control loop at {self.rate} Hz")
        
        # Performance monitoring
        loop_times = []
        max_loop_time = 1.0 / self.rate * 0.8  # 80% of period as warning threshold
        
        while not rospy.is_shutdown():
            loop_start = time.time()
            
            try:
                if self.emergency_stop:
                    rate.sleep()
                    continue
                    
                # Read motor states with timeout
                read_success = self.read_all_motor_states()
                
                if read_success:
                    self.update_robot_state()
                    self.publish_robot_state()
                    
                    # Send control commands (only if we have valid state)
                    if not self.calibration_mode:
                        self.send_control_commands()
                else:
                    rospy.logwarn_throttle(1.0, "Failed to read motor states")
                    
            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")
                self.execute_emergency_stop()
                break
                
            # Performance monitoring
            loop_time = time.time() - loop_start
            loop_times.append(loop_time)
            
            # Keep only last 100 measurements
            if len(loop_times) > 100:
                loop_times.pop(0)
                
            # Warn about timing issues
            if loop_time > max_loop_time:
                rospy.logwarn(f"Control loop timing violation: {loop_time*1000:.1f}ms (target: {max_loop_time*1000:.1f}ms)")
                
            # Log performance statistics every 10 seconds
            if len(loop_times) == 100:
                avg_time = sum(loop_times) / len(loop_times)
                max_time = max(loop_times)
                rospy.loginfo(f"Loop performance: avg={avg_time*1000:.1f}ms, max={max_time*1000:.1f}ms")
                loop_times = []
                
            rate.sleep()
            
    def shutdown(self):
        """Clean shutdown procedure"""
        rospy.loginfo("Shutting down exoskeleton motor controller")
        
        try:
            # Disable all motors
            if self.can_channel and self.motors_enabled:
                for joint_id in JointID:
                    controller_id = self.motor_controllers[joint_id].controller_id
                    exit_mode(self.can_channel, controller_id)
                    time.sleep(0.01)
                    
            # Close CAN communication
            if self.can_channel:
                self.can_channel.stop()
                rospy.loginfo("CAN communication closed")
                
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")

def main():
    try:
        controller = ExoskeletonMotorController()
        
        # Set shutdown hook
        rospy.on_shutdown(controller.shutdown)
        
        # Start control loop
        controller.control_loop()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")

if __name__ == '__main__':
    main()