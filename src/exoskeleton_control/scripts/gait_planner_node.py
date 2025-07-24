#!/usr/bin/env python

import rospy
from exoskeleton_control.msg import GaitParams, EStopTrigger
from exoskeleton_control.srv import InverseKinematicsClosedChain

class GaitPlannerNode:
    def __init__(self):
        rospy.init_node('gait_planner_node')

        # Subscribers
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)

        # Publishers
        self.gait_params_pub = rospy.Publisher('gait_params', GaitParams, queue_size=10)

        # Services
        rospy.wait_for_service('inverse_kinematics_closedchain')
        self.inverse_kinematics_closedchain_service = rospy.ServiceProxy('inverse_kinematics_closedchain', InverseKinematicsClosedChain)

        self.rate = rospy.Rate(10)  # 10-20Hz

    def e_stop_callback(self, msg):
        rospy.loginfo(f"Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            # Handle emergency stop
            pass

    def run(self):
        while not rospy.is_shutdown():
            # Your gait planning logic here
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GaitPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
