#!/usr/bin/env python3
"""
Test script for Emergency Stop Node functionality
Tests various emergency conditions and state transitions
"""

import rospy
import time
from exoskeleton_control.msg import MotorStatus, EStopTrigger, StopTrigger, CalibrationTrigger

class EmergencyStopTester:
    def __init__(self):
        rospy.init_node('emergency_stop_tester')
        
        # Publishers for testing
        self.motor_status_pub = rospy.Publisher('MotorStatus', MotorStatus, queue_size=1)
        self.stop_trigger_pub = rospy.Publisher('stop_trigger', StopTrigger, queue_size=1)
        self.calibration_trigger_pub = rospy.Publisher('calibration_trigger', CalibrationTrigger, queue_size=1)
        
        # Subscriber to monitor emergency stop responses
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)
        
        self.e_stop_received = False
        self.last_e_stop_msg = None
        
        rospy.loginfo("Emergency Stop Tester initialized")

    def e_stop_callback(self, msg):
        """Monitor emergency stop messages."""
        self.e_stop_received = True
        self.last_e_stop_msg = msg
        
        if msg.trigger:
            rospy.logwarn(f"🚨 EMERGENCY STOP TRIGGERED: {msg.reason}")
        else:
            rospy.loginfo(f"✅ Emergency stop cleared: {msg.reason}")

    def send_normal_motor_status(self):
        """Send normal motor status (no errors)."""
        msg = MotorStatus()
        msg.header.stamp = rospy.Time.now()
        msg.motor_ids = [1, 2, 3, 4]
        msg.positions = [0.1, -0.5, 0.1, -0.5]  # Normal joint positions
        msg.velocities = [0.0, 0.0, 0.0, 0.0]
        msg.torques = [5.0, 3.0, 5.0, 3.0]      # Normal torques
        msg.temperatures = [45.0, 40.0, 45.0, 40.0]  # Normal temperatures
        msg.errors = [0, 0, 0, 0]                # No errors
        
        self.motor_status_pub.publish(msg)

    def send_overtemperature_motor_status(self):
        """Send motor status with overtemperature condition."""
        msg = MotorStatus()
        msg.header.stamp = rospy.Time.now()
        msg.motor_ids = [1, 2, 3, 4]
        msg.positions = [0.1, -0.5, 0.1, -0.5]
        msg.velocities = [0.0, 0.0, 0.0, 0.0]
        msg.torques = [5.0, 3.0, 5.0, 3.0]
        msg.temperatures = [85.0, 40.0, 45.0, 40.0]  # Motor 1 overtemperature
        msg.errors = [0, 0, 0, 0]
        
        self.motor_status_pub.publish(msg)
        rospy.loginfo("Sent overtemperature motor status (Motor 1: 85°C)")

    def send_error_flag_motor_status(self):
        """Send motor status with error flags."""
        msg = MotorStatus()
        msg.header.stamp = rospy.Time.now()
        msg.motor_ids = [1, 2, 3, 4]
        msg.positions = [0.1, -0.5, 0.1, -0.5]
        msg.velocities = [0.0, 0.0, 0.0, 0.0]
        msg.torques = [5.0, 3.0, 5.0, 3.0]
        msg.temperatures = [45.0, 40.0, 45.0, 40.0]
        msg.errors = [0, 4, 0, 0]  # Motor 2 has error flag 4 (encoder error)
        
        self.motor_status_pub.publish(msg)
        rospy.loginfo("Sent motor status with error flag (Motor 2: encoder error)")

    def send_excessive_torque_motor_status(self):
        """Send motor status with excessive torque."""
        msg = MotorStatus()
        msg.header.stamp = rospy.Time.now()
        msg.motor_ids = [1, 2, 3, 4]
        msg.positions = [0.1, -0.5, 0.1, -0.5]
        msg.velocities = [0.0, 0.0, 0.0, 0.0]
        msg.torques = [5.0, 60.0, 5.0, 3.0]  # Motor 2 excessive torque
        msg.temperatures = [45.0, 40.0, 45.0, 40.0]
        msg.errors = [0, 0, 0, 0]
        
        self.motor_status_pub.publish(msg)
        rospy.loginfo("Sent excessive torque motor status (Motor 2: 60Nm)")

    def send_manual_stop_trigger(self):
        """Send manual stop trigger."""
        msg = StopTrigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        
        self.stop_trigger_pub.publish(msg)
        rospy.loginfo("Sent manual stop trigger")

    def send_calibration_trigger(self):
        """Send calibration trigger."""
        msg = CalibrationTrigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        
        self.calibration_trigger_pub.publish(msg)
        rospy.loginfo("Sent calibration trigger")

    def wait_for_e_stop(self, timeout=5.0):
        """Wait for emergency stop response."""
        self.e_stop_received = False
        start_time = time.time()
        
        while not self.e_stop_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        return self.e_stop_received

    def run_test_sequence(self):
        """Run comprehensive test sequence."""
        rospy.loginfo("="*60)
        rospy.loginfo("STARTING EMERGENCY STOP SYSTEM TESTS")
        rospy.loginfo("="*60)
        
        # Wait for system to be ready
        rospy.loginfo("Waiting for emergency stop node to be ready...")
        time.sleep(2.0)
        
        test_results = []
        
        # Test 1: Normal operation
        rospy.loginfo("\nTest 1: Normal motor status (should not trigger e-stop)")
        self.send_normal_motor_status()
        time.sleep(1.0)
        if not self.e_stop_received:
            rospy.loginfo("✅ PASS: Normal status did not trigger e-stop")
            test_results.append("PASS")
        else:
            rospy.logerr("❌ FAIL: Normal status incorrectly triggered e-stop")
            test_results.append("FAIL")
        
        # Test 2: Overtemperature condition
        rospy.loginfo("\nTest 2: Overtemperature condition (should trigger e-stop)")
        self.send_overtemperature_motor_status()
        if self.wait_for_e_stop(timeout=3.0):
            rospy.loginfo("✅ PASS: Overtemperature correctly triggered e-stop")
            test_results.append("PASS")
        else:
            rospy.logerr("❌ FAIL: Overtemperature did not trigger e-stop")
            test_results.append("FAIL")
        
        # Reset for next test
        time.sleep(2.0)
        
        # Test 3: Motor error flag
        rospy.loginfo("\nTest 3: Motor error flag (should trigger e-stop)")
        self.send_error_flag_motor_status()
        if self.wait_for_e_stop(timeout=3.0):
            rospy.loginfo("✅ PASS: Motor error flag correctly triggered e-stop")
            test_results.append("PASS")
        else:
            rospy.logerr("❌ FAIL: Motor error flag did not trigger e-stop")
            test_results.append("FAIL")
        
        # Reset for next test
        time.sleep(2.0)
        
        # Test 4: Excessive torque
        rospy.loginfo("\nTest 4: Excessive torque (should trigger e-stop)")
        self.send_excessive_torque_motor_status()
        if self.wait_for_e_stop(timeout=3.0):
            rospy.loginfo("✅ PASS: Excessive torque correctly triggered e-stop")
            test_results.append("PASS")
        else:
            rospy.logerr("❌ FAIL: Excessive torque did not trigger e-stop")
            test_results.append("FAIL")
        
        # Reset for next test
        time.sleep(2.0)
        
        # Test 5: Manual stop trigger
        rospy.loginfo("\nTest 5: Manual stop trigger (should trigger e-stop)")
        self.send_manual_stop_trigger()
        if self.wait_for_e_stop(timeout=3.0):
            rospy.loginfo("✅ PASS: Manual stop correctly triggered e-stop")
            test_results.append("PASS")
        else:
            rospy.logerr("❌ FAIL: Manual stop did not trigger e-stop")
            test_results.append("FAIL")
        
        # Test Summary
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST RESULTS SUMMARY")
        rospy.loginfo("="*60)
        
        test_names = [
            "Normal Operation",
            "Overtemperature Detection",
            "Error Flag Detection", 
            "Excessive Torque Detection",
            "Manual Stop Trigger"
        ]
        
        passed = 0
        for i, (name, result) in enumerate(zip(test_names, test_results)):
            status = "✅ PASS" if result == "PASS" else "❌ FAIL"
            rospy.loginfo(f"Test {i+1}: {name:<25} {status}")
            if result == "PASS":
                passed += 1
        
        rospy.loginfo("-"*60)
        rospy.loginfo(f"Overall: {passed}/{len(test_results)} tests passed")
        
        if passed == len(test_results):
            rospy.loginfo("🎉 ALL TESTS PASSED - Emergency Stop System Working Correctly!")
        else:
            rospy.logerr("⚠️  SOME TESTS FAILED - Check Emergency Stop Configuration")
        
        rospy.loginfo("="*60)

    def run(self):
        """Main execution method."""
        rospy.loginfo("Emergency Stop Tester ready")
        rospy.loginfo("Starting test sequence in 3 seconds...")
        time.sleep(3.0)
        
        self.run_test_sequence()
        
        rospy.loginfo("Test sequence complete. Keeping node alive for manual testing...")
        rospy.loginfo("Use Ctrl+C to exit")
        
        # Keep node alive for manual testing
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Emergency Stop Tester shutdown")

if __name__ == '__main__':
    try:
        tester = EmergencyStopTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Emergency Stop Tester interrupted")
    except Exception as e:
        rospy.logerr(f"Error in Emergency Stop Tester: {e}")
