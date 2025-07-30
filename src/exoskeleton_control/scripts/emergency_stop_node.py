#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import threading
import time
from exoskeleton_control.msg import MotorStatus, EStopTrigger, Trigger, ExoskeletonState, CrutchCommand

class SystemStates:
    """System state constants"""
    INIT = "INIT"
    CALIBRATION_PROCESS = "CALIBRATION_PROCESS"
    READY = "READY"
    WALKING = "WALKING"
    STOPPING = "STOPPING"
    E_STOP = "E_STOP"

class EmergencyStopNode:
    """
    Emergency Stop Node with SMACH state machine for exoskeleton safety management.
    Monitors motor status, user triggers, and system conditions to ensure safe operation.
    """
    
    def __init__(self):
        rospy.init_node('emergency_stop_node')
        
        # Load configuration
        self.load_configuration()
        
        # State management
        self.current_state = SystemStates.INIT
        self.emergency_active = False
        self.motor_data = {}
        self.last_motor_update = rospy.Time.now()
        self.calibration_complete = False
        self.calibration_failed = False
        self.cycle_finished = False
        
        # Crutch command triggers
        self.st_calibration_trig = False
        self.st_walking_trig = False
        self.stop_trig = False
        self.shutdown_trig = False
        
        # Thread safety
        self.state_lock = threading.Lock()
        
        # Subscribers
        rospy.Subscriber('Motor_Status', MotorStatus, self.motor_status_callback)
        rospy.Subscriber('crutch_command', CrutchCommand, self.crutch_command_callback)
        rospy.Subscriber('cycle_finished', Trigger, self.cycle_finished_callback)
        
        # Publishers
        self.e_stop_trigger_pub = rospy.Publisher('e_stop_trigger', EStopTrigger, queue_size=1)
        self.calibration_trigger_pub = rospy.Publisher('calibration_trigger_fw', Trigger, queue_size=1)
        self.walking_trigger_pub = rospy.Publisher('walking_trigger', Trigger, queue_size=1)
        self.stopping_trigger_pub = rospy.Publisher('stopping_trigger', Trigger, queue_size=1)
        self.system_state_pub = rospy.Publisher('system_state', Trigger, queue_size=1)
        
        # Timer for periodic state publishing
        self.state_publish_timer = rospy.Timer(rospy.Duration(0.1), self.publish_state_periodically)
        
        # Create SMACH state machine
        self.create_state_machine()
        
        # Create and start the introspection server for visualization
        if self.enable_introspection:
            self.sis = smach_ros.IntrospectionServer('emergency_stop_smach', self.sm, '/EMERGENCY_STOP_SM')
            self.sis.start()
            rospy.loginfo("SMACH introspection server started")
        
        # State machine execution thread
        self.sm_thread = threading.Thread(target=self.run_state_machine)
        self.sm_thread.daemon = True
        
        # Monitoring timer
        self.monitor_timer = rospy.Timer(rospy.Duration(1.0 / self.monitor_frequency), self.monitor_callback)
        
        rospy.loginfo("Emergency Stop Node initialized")
        rospy.loginfo(f"Configuration: max_temp={self.max_motor_temperature}°C, "
                     f"timeout={self.communication_timeout}s, "
                     f"monitor_freq={self.monitor_frequency}Hz")

    def load_configuration(self):
        """Load configuration parameters from ROS parameter server."""
        try:
            # Monitoring parameters
            self.monitor_frequency = rospy.get_param('~monitor_frequency', 10.0)  # Hz
            self.communication_timeout = rospy.get_param('~communication_timeout', 2.0)  # seconds
            
            # Safety thresholds
            self.max_motor_temperature = rospy.get_param('~safety/max_motor_temperature', 80.0)  # Celsius
            self.max_torque_error = rospy.get_param('~safety/max_torque_error', 50.0)  # Nm
            self.max_position_error = rospy.get_param('~safety/max_position_error', 1.57)  # rad (90 degrees)
            self.critical_error_flags = rospy.get_param('~safety/critical_error_flags', [1, 2, 4, 8])  # Bit flags
            
            # State machine parameters
            self.auto_recovery_enabled = rospy.get_param('~state_machine/auto_recovery', False)
            self.error_recovery_delay = rospy.get_param('~state_machine/error_recovery_delay', 5.0)  # seconds
            self.enable_introspection = rospy.get_param('~state_machine/enable_introspection', True)
            
            # Emergency stop behavior
            self.auto_stop_on_error = rospy.get_param('~emergency/auto_stop_on_error', True)
            self.stop_on_communication_loss = rospy.get_param('~emergency/stop_on_communication_loss', True)
            self.stop_on_temperature = rospy.get_param('~emergency/stop_on_temperature', True)
            
            rospy.loginfo("Emergency stop configuration loaded successfully")
            
        except Exception as e:
            rospy.logerr(f"Error loading configuration: {e}")
            self.set_default_configuration()

    def set_default_configuration(self):
        """Set default configuration values."""
        self.monitor_frequency = 10.0
        self.communication_timeout = 2.0
        self.max_motor_temperature = 80.0
        self.max_torque_error = 50.0
        self.max_position_error = 1.57
        self.critical_error_flags = [1, 2, 4, 8]
        self.auto_recovery_enabled = False
        self.error_recovery_delay = 5.0
        self.enable_introspection = True
        self.auto_stop_on_error = True
        self.stop_on_communication_loss = True
        self.stop_on_temperature = True

    def create_state_machine(self):
        """Create and configure the SMACH state machine."""
        self.sm = smach.StateMachine(outcomes=['shutdown'])
        
        with self.sm:
            # Add states with their transitions
            smach.StateMachine.add(
                'INIT', 
                InitState(self), 
                transitions={
                    'calibration_process': 'CALIBRATION_PROCESS',
                    'e_stop': 'E_STOP',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'CALIBRATION_PROCESS', 
                CalibrationProcessState(self), 
                transitions={
                    'init': 'INIT',
                    'ready': 'READY',
                    'e_stop': 'E_STOP',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'READY', 
                ReadyState(self), 
                transitions={
                    'walking': 'WALKING',
                    'e_stop': 'E_STOP',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'WALKING', 
                WalkingState(self), 
                transitions={
                    'stopping': 'STOPPING',
                    'e_stop': 'E_STOP',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'STOPPING', 
                StoppingState(self), 
                transitions={
                    'ready': 'READY',
                    'e_stop': 'E_STOP',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'E_STOP', 
                EStopState(self), 
                transitions={
                    'shutdown': 'shutdown'
                }
            )

    def motor_status_callback(self, msg):
        """Process motor status messages and check for errors."""
        current_time = rospy.Time.now()
        
        with self.state_lock:
            for i, motor_id in enumerate(msg.motor_ids):
                self.motor_data[motor_id] = {
                    'position': msg.positions[i],
                    'velocity': msg.velocities[i],
                    'torque': msg.torques[i],
                    'temperature': msg.temperatures[i],
                    'error_flags': msg.error_flags[i],
                    'last_update': current_time
                }
            self.calibration_complete = all(msg.calibrated_flags)
            # Check for calibration failure
            if hasattr(msg, 'calibration_failed'):
                self.calibration_failed = any(msg.calibration_failed)
            self.last_motor_update = current_time
            
            # Check for immediate safety issues
            self.check_motor_safety(msg)

    def crutch_command_callback(self, msg):
        """Handle crutch commands from dummy_crutches_node."""
        with self.state_lock:
            if msg.command == CrutchCommand.ST_CALIBRATION_TRIG:
                rospy.loginfo("Crutch command: Start calibration")
                self.st_calibration_trig = True
            elif msg.command == CrutchCommand.ST_WALKING_TRIG:
                rospy.loginfo("Crutch command: Start walking")
                self.st_walking_trig = True
            elif msg.command == CrutchCommand.STOP_TRIG:
                rospy.loginfo("Crutch command: Stop")
                self.stop_trig = True
            elif msg.command == CrutchCommand.SHUTDOWN:
                rospy.logwarn("Crutch command: Emergency shutdown")
                self.shutdown_trig = True
                self.trigger_emergency_stop("Shutdown command from crutches")
            elif msg.command == CrutchCommand.DISABLE_MOTORS:
                rospy.loginfo("Crutch command: Disable motors (not implemented)")
            elif msg.command == CrutchCommand.BREAK_MOTORS:
                rospy.loginfo("Crutch command: Break motors (not implemented)")

    def cycle_finished_callback(self, msg):
        """Handle cycle finished signal from trajectory generator."""
        if msg.trigger:
            rospy.loginfo("Cycle finished signal received")
            with self.state_lock:
                self.cycle_finished = True

    def check_motor_safety(self, msg):
        """Check motor status for safety violations."""
        if not self.auto_stop_on_error:
            return
            
        emergency_reasons = []
        
        # Check error flags
        if hasattr(msg, 'errors'):
            for i, error_flag in enumerate(msg.errors):
                if error_flag != 0:
                    for critical_flag in self.critical_error_flags:
                        if error_flag & critical_flag:
                            emergency_reasons.append(f"Motor {i+1} critical error flag: {error_flag}")
        
        # Check temperatures
        if self.stop_on_temperature and hasattr(msg, 'temperatures'):
            for i, temp in enumerate(msg.temperatures):
                if temp > self.max_motor_temperature:
                    emergency_reasons.append(f"Motor {i+1} overtemperature: {temp:.1f}°C")
        
        # Check torques (if available)
        if hasattr(msg, 'torques'):
            for i, torque in enumerate(msg.torques):
                if abs(torque) > self.max_torque_error:
                    emergency_reasons.append(f"Motor {i+1} excessive torque: {torque:.1f}Nm")
        
        # Trigger emergency stop if issues found
        if emergency_reasons:
            reason = "; ".join(emergency_reasons)
            rospy.logerr(f"Safety violation detected: {reason}")
            self.trigger_emergency_stop(reason)

    def monitor_callback(self, event):
        """Periodic monitoring for communication timeouts and system health."""
        current_time = rospy.Time.now()
        
        # Check communication timeout (skip during calibration)
        if self.stop_on_communication_loss and self.current_state != SystemStates.CALIBRATION_PROCESS:
            time_since_update = (current_time - self.last_motor_update).to_sec()
            if time_since_update > self.communication_timeout:
                rospy.logwarn(f"Motor communication timeout: {time_since_update:.1f}s")
                self.trigger_emergency_stop(f"Communication timeout: {time_since_update:.1f}s")

    def trigger_emergency_stop(self, reason="Emergency stop activated"):
        """Trigger emergency stop and publish e-stop message."""
        with self.state_lock:
            if not self.emergency_active:
                self.emergency_active = True
                rospy.logwarn(f"EMERGENCY STOP TRIGGERED: {reason}")
                
                # Update state to E_STOP
                self.current_state = SystemStates.E_STOP
                
                # Publish emergency stop message
                e_stop_msg = EStopTrigger()
                e_stop_msg.header.stamp = rospy.Time.now()
                e_stop_msg.trigger = True
                e_stop_msg.state = self.current_state
                
                self.e_stop_trigger_pub.publish(e_stop_msg)
                
                # Schedule shutdown after a brief delay to ensure message is sent
                def delayed_shutdown():
                    rospy.sleep(0.5)  # Give time for message to be sent and processed
                    rospy.logwarn("Emergency stop complete - shutting down emergency stop node")
                    rospy.signal_shutdown("Emergency stop triggered")
                
                shutdown_thread = threading.Thread(target=delayed_shutdown)
                shutdown_thread.daemon = True
                shutdown_thread.start()

    def clear_emergency_stop(self, reason="Emergency cleared"):
        """Clear emergency stop condition."""
        with self.state_lock:
            if self.emergency_active:
                self.emergency_active = False
                rospy.loginfo(f"Emergency stop cleared: {reason}")
                
                # Publish clear message
                e_stop_msg = EStopTrigger()
                e_stop_msg.header.stamp = rospy.Time.now()
                e_stop_msg.trigger = False
                e_stop_msg.state = self.current_state
                
                self.e_stop_trigger_pub.publish(e_stop_msg)
                
    def publish_current_state(self):
        """Publish current system state to motor control nodes."""
        e_stop_msg = EStopTrigger()
        e_stop_msg.header.stamp = rospy.Time.now()
        e_stop_msg.trigger = self.emergency_active
        e_stop_msg.state = self.current_state
        self.e_stop_trigger_pub.publish(e_stop_msg)
        
    def publish_state_periodically(self, event):
        """Periodically publish state for motor control nodes."""
        if not rospy.is_shutdown():
            self.publish_current_state()

    def update_state(self, new_state):
        """Update current system state."""
        with self.state_lock:
            if self.current_state != new_state:
                rospy.loginfo(f"State transition: {self.current_state} -> {new_state}")
                old_state = self.current_state
                self.current_state = new_state
                
                # Send appropriate triggers to motor control nodes
                self.send_state_triggers(old_state, new_state)
                
                # Always publish current state for motor control nodes
                self.publish_current_state()

    def send_walking_trigger(self):
        """Send walking trigger to motor control nodes."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.walking_trigger_pub.publish(msg)
        rospy.loginfo("Walking trigger sent to motor control nodes")

    def send_stopping_trigger(self):
        """Send stopping trigger to motor control nodes."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        self.stopping_trigger_pub.publish(msg)
        rospy.loginfo("Stopping trigger sent to motor control nodes")

    def send_system_state(self, state_name):
        """Send system state update to motor control nodes."""
        msg = Trigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        # Note: Using trigger for simplicity, could create custom message with state field
        self.system_state_pub.publish(msg)
        rospy.loginfo(f"System state '{state_name}' sent to motor control nodes")

    def send_state_triggers(self, old_state, new_state):
        """Send appropriate triggers based on state transitions."""
        # Send walking trigger when entering WALKING state
        if new_state == SystemStates.WALKING and old_state == SystemStates.READY:
            self.send_walking_trigger()
            
        # Send stopping trigger when entering STOPPING state  
        elif new_state == SystemStates.STOPPING and old_state == SystemStates.WALKING:
            self.send_stopping_trigger()
            
        # Always send state update
        self.send_system_state(new_state)

    def run_state_machine(self):
        """Run the SMACH state machine in a separate thread."""
        try:
            outcome = self.sm.execute()
            rospy.loginfo(f"State machine completed with outcome: {outcome}")
        except Exception as e:
            rospy.logerr(f"State machine execution error: {e}")

    def shutdown(self):
        """Cleanup method called on node shutdown."""
        rospy.loginfo("Shutting down Emergency Stop Node...")
        
        # Stop monitoring timer
        if hasattr(self, 'monitor_timer'):
            self.monitor_timer.shutdown()
            
        # Stop state publish timer
        if hasattr(self, 'state_publish_timer'):
            self.state_publish_timer.shutdown()
        
        # Cleanup SMACH introspection
        if hasattr(self, 'sis'):
            self.sis.stop()
            
        rospy.loginfo("Emergency Stop Node shutdown complete")

    def run(self):
        """Main execution method."""
        rospy.loginfo("Starting Emergency Stop Node...")
        
        # Register shutdown handler
        rospy.on_shutdown(self.shutdown)
        
        # Start state machine
        self.sm_thread.start()
        
        # Keep node alive
        rospy.spin()

# SMACH State Definitions
class InitState(smach.State):
    """Initial state - waiting for calibration trigger."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['calibration_process', 'e_stop', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.INIT)
        rospy.loginfo('System in INIT state - waiting for calibration')
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            with self.node.state_lock:
                if self.node.st_calibration_trig:
                    rospy.loginfo('Calibration trigger received - starting calibration')
                    self.node.st_calibration_trig = False  # Reset trigger
                    return 'calibration_process'
                    
                if self.node.shutdown_trig or self.node.emergency_active:
                    return 'e_stop'
                
            rate.sleep()
        
        return 'shutdown'

class CalibrationProcessState(smach.State):
    """Calibration process state - monitoring calibration progress."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['init', 'ready', 'e_stop', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.CALIBRATION_PROCESS)
        rospy.loginfo('System in CALIBRATION_PROCESS state')
        
        # Send calibration trigger once
        calibration_msg = Trigger()
        calibration_msg.header.stamp = rospy.Time.now()
        calibration_msg.trigger = True
        self.node.calibration_trigger_pub.publish(calibration_msg)
        rospy.loginfo('Calibration trigger sent to motor control node')

        rate = rospy.Rate(10)  # 10 Hz
        calibration_start_time = rospy.Time.now()
        max_calibration_time = 120.0  # 2 minutes maximum
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = (current_time - calibration_start_time).to_sec()
            
            with self.node.state_lock:
                # Check for calibration failure
                if self.node.calibration_failed:
                    rospy.logwarn("Calibration failed - returning to INIT")
                    self.node.calibration_failed = False  # Reset flag
                    return 'init'
                    
                # Check for calibration completion
                if self.node.calibration_complete:
                    rospy.loginfo("Calibration completed successfully")
                    return 'ready'
                    
                # Check for shutdown command or emergency
                if self.node.shutdown_trig or self.node.emergency_active:
                    return 'e_stop'
            
            # Check for calibration timeout
            if elapsed_time > max_calibration_time:
                rospy.logerr("Calibration timeout")
                self.node.trigger_emergency_stop("Calibration timeout")
                return 'e_stop'
            
            rate.sleep()
        
        return 'shutdown'

class ReadyState(smach.State):
    """Ready state - waiting for walking command."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['walking', 'e_stop', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.READY)
        rospy.loginfo('System READY for operation')
        
        # Clear any previous emergency stop
        self.node.clear_emergency_stop("System ready")
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            with self.node.state_lock:
                # Check for walking trigger
                if self.node.st_walking_trig:
                    rospy.loginfo('Walking trigger received')
                    self.node.st_walking_trig = False  # Reset trigger
                    return 'walking'
                    
                # Check for shutdown command or emergency
                if self.node.shutdown_trig or self.node.emergency_active:
                    return 'e_stop'
            
            rate.sleep()
        
        return 'shutdown'

class WalkingState(smach.State):
    """Walking state - exoskeleton is actively walking."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['stopping', 'e_stop', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.WALKING)
        rospy.loginfo('System WALKING - active gait execution')
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            with self.node.state_lock:
                # Check for stop trigger
                if self.node.stop_trig:
                    rospy.loginfo('Stop trigger received - transitioning to stopping')
                    self.node.stop_trig = False  # Reset trigger
                    return 'stopping'
                    
                # Check for shutdown command or emergency
                if self.node.shutdown_trig or self.node.emergency_active:
                    return 'e_stop'
            
            rate.sleep()
        
        return 'shutdown'

class StoppingState(smach.State):
    """Stopping state - transitioning from walking to ready."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['ready', 'e_stop', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.STOPPING)
        rospy.loginfo('System STOPPING - waiting for cycle completion')
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            with self.node.state_lock:
                # Check for cycle finished signal
                if self.node.cycle_finished:
                    rospy.loginfo('Cycle finished - returning to ready')
                    self.node.cycle_finished = False  # Reset flag
                    return 'ready'
                    
                # Check for shutdown command or emergency
                if self.node.shutdown_trig or self.node.emergency_active:
                    return 'e_stop'
            
            rate.sleep()
        
        return 'shutdown'

class EStopState(smach.State):
    """Emergency stop state - system fault/shutdown condition."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.E_STOP)
        rospy.logerr('System in E_STOP state - shutting down')
        
        # Ensure emergency stop is active and published
        self.node.trigger_emergency_stop("System in E_STOP state")
        
        # Wait briefly to ensure message is sent, then shutdown
        rospy.sleep(1.0)
        
        return 'shutdown'

if __name__ == '__main__':
    try:
        node = EmergencyStopNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Emergency Stop Node shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error in Emergency Stop Node: {e}")
