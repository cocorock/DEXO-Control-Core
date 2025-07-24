#!/usr/bin/env python

import rospy
from exoskeleton_control.msg import StopTrigger, CalibrationTrigger

def dummy_crutches():
    rospy.init_node('dummy_crutches', anonymous=True)
    stop_pub = rospy.Publisher('stop_trigger', StopTrigger, queue_size=10)
    calib_pub = rospy.Publisher('calibration_trigger', CalibrationTrigger, queue_size=10)

    while not rospy.is_shutdown():
        command = raw_input("Enter 's' to send stop trigger, 'c' to send calibration trigger: ")
        if command == 's':
            stop_msg = StopTrigger()
            stop_msg.trigger = True
            stop_pub.publish(stop_msg)
            rospy.loginfo("Stop trigger sent.")
        elif command == 'c':
            calib_msg = CalibrationTrigger()
            calib_msg.trigger = True
            calib_pub.publish(calib_msg)
            rospy.loginfo("Calibration trigger sent.")

if __name__ == '__main__':
    try:
        dummy_crutches()
    except rospy.ROSInterruptException:
        pass
