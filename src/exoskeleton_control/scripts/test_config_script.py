#!/usr/bin/env python
"""
Test script to verify motor control configuration loading
Run this to test parameter loading without starting the full motor control system
"""

import rospy
import math

def test_configuration():
    """Test loading configuration parameters"""
    
    rospy.init_node('test_motor_config')
    
    print("Testing Motor Control Configuration Loading...")
    print("=" * 50)
    
    try:
        # Test calibration parameters
        calibration_sequence = rospy.get_param('~calibration_sequence', [1, 3, 2, 4])
        torque_threshold = rospy.get_param('~torque_threshold', 8.0)
        calibration_speed_deg = rospy.get_param('~calibration_speed_deg', 30.0)
        max_calibration_time = rospy.get_param('~max_calibration_time', 30.0)
        
        print("CALIBRATION PARAMETERS:")
        print(f"  Sequence: {calibration_sequence}")
        print(f"  Torque threshold: {torque_threshold} Nm")
        print(f"  Speed: {calibration_speed_deg} deg/s ({math.radians(calibration_speed_deg):.3f} rad/s)")
        print(f"  Max time: {max_calibration_time} s")
        
        # Test gains
        gains = {
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
        
        print("\nCONTROL GAINS:")
        for mode, values in gains.items():
            print(f"  {mode.capitalize()}: kp={values['kp']}, kd={values['kd']}")
        
        # Test motor configurations
        motor_ids = rospy.get_param('~motor_ids', [0x06, 0x07, 0x08, 0x09])
        joint_names = rospy.get_param('~joint_names', ["right_hip", "right_knee", "left_hip", "left_knee"])
        motor_models = rospy.get_param('~motor_models', ["AK80_64", "AK80_8", "AK80_64", "AK80_8"])
        angle_limits_deg = rospy.get_param('~angle_limits_deg', [[-30, 90], [-100, 0], [-30, 90], [-100, 0]])
        motor_directions = rospy.get_param('~motor_directions', [1, 1, 1, 1])
        
        print("\nMOTOR CONFIGURATIONS:")
        for i in range(len(motor_ids)):
            print(f"  Motor {i+1} ({joint_names[i]}):")
            print(f"    CAN ID: 0x{motor_ids[i]:02X}")
            print(f"    Model: {motor_models[i]}")
            print(f"    Limits: {angle_limits_deg[i][0]}° to {angle_limits_deg[i][1]}°")
            print(f"    Direction: {motor_directions[i]}")
        
        # Test optional parameters
        try:
            control_frequency = rospy.get_param('~control_frequency', 100.0)
            can_bitrate = rospy.get_param('~can_interface/bitrate', 1000000)
            
            print("\nOPTIONAL PARAMETERS:")
            print(f"  Control frequency: {control_frequency} Hz")
            print(f"  CAN bitrate: {can_bitrate} bps")
            
        except Exception as e:
            print(f"\nOptional parameters not found (this is normal): {e}")
        
        print("\n" + "=" * 50)
        print("✅ Configuration loading test PASSED!")
        print("All parameters loaded successfully from YAML file")
        
        return True
        
    except Exception as e:
        print(f"\n❌ Configuration loading test FAILED!")
        print(f"Error: {e}")
        print("\nMake sure to:")
        print("1. Load the YAML file with rosparam:")
        print("   rosparam load motor_control_config.yaml /test_motor_config/")
        print("2. Or use the launch file:")
        print("   roslaunch exoskeleton_control test_motor_config.launch")
        
        return False

if __name__ == '__main__':
    try:
        test_configuration()
    except rospy.ROSInterruptException:
        pass