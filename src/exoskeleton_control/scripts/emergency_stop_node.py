#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import threading
import time
from exoskeleton_control.msg import MotorStatus, EStopTrigger, StopTrigger, CalibrationTrigger, CalibrationTriggerFw, ExoskeletonState

class SystemStates:
    """System state constants"""
    INIT = "INIT"
    CALIBRATING = "CALIBRATING" 
    READY = "READY"
    STOPPED = "STOPPED"
    ERROR = "ERROR"

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
        self.manual_stop_received = False
        self.manual_calibration_trigger = False
        self.calibration_complete = False
        
        # Thread safety
        self.state_lock = threading.Lock()
        
        # Subscribers
        rospy.Subscriber('Motor_Status', MotorStatus, self.motor_status_callback)
        rospy.Subscriber('stop_trigger', StopTrigger, self.stop_trigger_callback)
        rospy.Subscriber('manual_calibration_trigger', CalibrationTrigger, self.calibration_trigger_callback)
        
        # Publishers
        self.e_stop_trigger_pub = rospy.Publisher('e_stop_trigger', EStopTrigger, queue_size=1)
        self.calibration_trigger_pub = rospy.Publisher('calibration_trigger_fw', CalibrationTriggerFw, queue_size=1)
        
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
                    'start_calibration': 'CALIBRATING',
                    'error': 'ERROR',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'CALIBRATING', 
                CalibratingState(self), 
                transitions={
                    'calibration_complete': 'READY',
                    'error': 'ERROR',
                    'stop': 'STOPPED',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'READY', 
                ReadyState(self), 
                transitions={
                    'stop': 'STOPPED',
                    'error': 'ERROR',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'STOPPED', 
                StoppedState(self), 
                transitions={
                    'resume': 'READY',
                    'restart_calibration': 'CALIBRATING',
                    'error': 'ERROR',
                    'shutdown': 'shutdown'
                }
            )
            
            smach.StateMachine.add(
                'ERROR', 
                ErrorState(self), 
                transitions={
                    'recovery': 'INIT',
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
            self.last_motor_update = current_time
            
            # Check for immediate safety issues
            self.check_motor_safety(msg)

    def stop_trigger_callback(self, msg):
        """Handle manual stop trigger."""
        if msg.trigger:
            rospy.logwarn("Manual stop trigger received")
            with self.state_lock:
                self.manual_stop_received = True
            self.trigger_emergency_stop("Manual stop triggered")

    def calibration_trigger_callback(self, msg):
        """Handle manual calibration trigger."""
        if msg.trigger:
            rospy.loginfo("Manual calibration trigger received")
            with self.state_lock:
                self.manual_calibration_trigger = True

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
        if self.stop_on_communication_loss and self.current_state != SystemStates.CALIBRATING:
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
                
                # Publish emergency stop message
                e_stop_msg = EStopTrigger()
                e_stop_msg.header.stamp = rospy.Time.now()
                e_stop_msg.trigger = True
                e_stop_msg.state = self.current_state

                
                self.e_stop_trigger_pub.publish(e_stop_msg)

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

    def update_state(self, new_state):
        """Update current system state."""
        with self.state_lock:
            if self.current_state != new_state:
                rospy.loginfo(f"State transition: {self.current_state} -> {new_state}")
                self.current_state = new_state

    def run_state_machine(self):
        """Run the SMACH state machine in a separate thread."""
        try:
            outcome = self.sm.execute()
            rospy.loginfo(f"State machine completed with outcome: {outcome}")
        except Exception as e:
            rospy.logerr(f"State machine execution error: {e}")

    def run(self):
        """Main execution method."""
        rospy.loginfo("Starting Emergency Stop Node...")
        
        # Start state machine
        self.sm_thread.start()
        
        # Keep node alive
        rospy.spin()
        
        # Cleanup
        if hasattr(self, 'sis'):
            self.sis.stop()

# SMACH State Definitions
class InitState(smach.State):
    """Initial state - waiting for calibration trigger."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start_calibration', 'error', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.INIT)
        rospy.loginfo('System in INIT state - waiting for calibration')
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            with self.node.state_lock:
                if self.node.manual_calibration_trigger:
                    rospy.loginfo('Calibration trigger received - starting calibration')
                    self.node.manual_calibration_trigger = False  # Reset trigger
                    return 'start_calibration'
            
            if self.node.emergency_active:
                return 'error'
                
            rate.sleep()
        
        return 'shutdown'

class CalibratingState(smach.State):
    """Calibration state - monitoring calibration progress."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['calibration_complete', 'error', 'stop', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.CALIBRATING)
        rospy.loginfo('System in CALIBRATING state')
        
        # Send calibration trigger once
        calibration_msg = CalibrationTriggerFw()
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
            
            # Check for calibration completion
            with self.node.state_lock:
                if self.node.calibration_complete:
                    rospy.loginfo("Calibration completed successfully")
                    return 'calibration_complete'
            
            # Check for emergency stop
            if self.node.emergency_active:
                return 'error'
            
            # Check for manual stop
            with self.node.state_lock:
                if self.node.manual_stop_received:
                    self.node.manual_stop_received = False
                    return 'stop'
            
            # Check for calibration timeout
            if elapsed_time > max_calibration_time:
                rospy.logerr("Calibration timeout")
                self.node.trigger_emergency_stop("Calibration timeout")
                return 'error'
            
            rate.sleep()
        
        return 'shutdown'

class ReadyState(smach.State):
    """Ready state - normal operation mode."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['stop', 'error', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.READY)
        rospy.loginfo('System READY for operation')
        
        # Clear any previous emergency stop
        self.node.clear_emergency_stop("System ready")
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Check for emergency conditions
            if self.node.emergency_active:
                return 'error'
            
            # Check for manual stop
            with self.node.state_lock:
                if self.node.manual_stop_received:
                    self.node.manual_stop_received = False
                    return 'stop'
            
            rate.sleep()
        
        return 'shutdown'

class StoppedState(smach.State):
    """Stopped state - manual stop activated."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['resume', 'restart_calibration', 'error', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.STOPPED)
        rospy.loginfo('System STOPPED - waiting for resume command')
        
        rate = rospy.Rate(10)  # 10 Hz
        stop_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Check for emergency conditions
            if self.node.emergency_active:
                return 'error'
            
            # Check if calibration is lost (would require recalibration)
            with self.node.state_lock:
                if not self.node.calibration_complete:
                    rospy.logwarn("Calibration lost during stop - restart required")
                    return 'restart_calibration'
            
            # For now, auto-resume after a delay (in real implementation, 
            # this would be triggered by user command)
            elapsed_time = (rospy.Time.now() - stop_start_time).to_sec()
            if elapsed_time > 5.0:  # Auto-resume after 5 seconds for testing
                rospy.loginfo("Auto-resuming from stop state")
                return 'resume'
            
            rate.sleep()
        
        return 'shutdown'

class ErrorState(smach.State):
    """Error state - system fault condition."""
    
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['recovery', 'shutdown'])
        self.node = node
    
    def execute(self, userdata):
        self.node.update_state(SystemStates.ERROR)
        rospy.logerr('System in ERROR state - manual intervention required')
        
        rate = rospy.Rate(1)  # 1 Hz (slower rate in error state)
        error_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Check for recovery conditions
            if self.node.auto_recovery_enabled:
                elapsed_time = (rospy.Time.now() - error_start_time).to_sec()
                if elapsed_time > self.node.error_recovery_delay:
                    rospy.loginfo("Attempting automatic recovery")
                    # Clear emergency condition
                    self.node.clear_emergency_stop("Automatic recovery attempt")
                    return 'recovery'
            
            # Manual recovery would be triggered by operator action
            # For now, stay in error state indefinitely
            
            rate.sleep()
        
        return 'shutdown'

if __name__ == '__main__':
    try:
        node = EmergencyStopNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Emergency Stop Node shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error in Emergency Stop Node: {e}")
