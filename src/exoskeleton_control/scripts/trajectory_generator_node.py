#!/usr/bin/env python

import rospy
import json
from exoskeleton_control.msg import GaitParams, JointsTrajectory, EStopTrigger
from exoskeleton_control.srv import InverseKinematicsLeg, ForwardKinematicsLeg

class TrajectoryGeneratorNode:
    def __init__(self):
        rospy.init_node('trajectory_generator_node')

        # Subscribers
        rospy.Subscriber('gait_params', GaitParams, self.gait_params_callback)
        rospy.Subscriber('e_stop_trigger', EStopTrigger, self.e_stop_callback)

        # Publishers
        self.joints_trajectory_pub = rospy.Publisher('joints_trajectory', JointsTrajectory, queue_size=10)

        # Services
        rospy.wait_for_service('inverse_kinematics_leg')
        self.inverse_kinematics_leg_service = rospy.ServiceProxy('inverse_kinematics_leg', InverseKinematicsLeg)
        rospy.wait_for_service('forward_kinematics_leg')
        self.forward_kinematics_leg_service = rospy.ServiceProxy('forward_kinematics_leg', ForwardKinematicsLeg)

        self.rate = rospy.Rate(15)  # 15-20Hz

        # Load trajectory from JSON for testing
        self.trajectory = self.load_trajectory_from_json('trajectory.json')

    def load_trajectory_from_json(self, file_path):
        try:
            with open(file_path, 'r') as f:
                return json.load(f)
        except IOError as e:
            rospy.logerr(f"Error loading trajectory from {file_path}: {e}")
            return None

    def gait_params_callback(self, msg):
        # Process gait parameters
        pass

    def e_stop_callback(self, msg):
        rospy.loginfo(f"Received e_stop_trigger: {msg.trigger}, state: {msg.state}")
        if msg.trigger:
            # Handle emergency stop
            pass

    def run(self):
        while not rospy.is_shutdown():
            if self.trajectory:
                # Publish trajectory from JSON
                trajectory_msg = JointsTrajectory()
                # Populate trajectory_msg from self.trajectory
                self.joints_trajectory_pub.publish(trajectory_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TrajectoryGeneratorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
