#!/usr/bin/env python

import rospy
import math
from exoskeleton_control.srv import ForwardKinematicsLeg, ForwardKinematicsLegResponse

class ForwardKinematicsService:
    def __init__(self):
        # Arm parameters (you can adjust these based on your robot)
        self.L1 = 0.39  # Length of thigh (first link) in meters
        self.L2 = 0.42  # Length of shin (second link) in meters
        
        # Initialize the service
        self.service = rospy.Service('forward_kinematics_leg', ForwardKinematicsLeg, self.calculate_forward_kinematics)
        rospy.loginfo("Forward Kinematics Service Ready")
    
    def calculate_forward_kinematics(self, req):
        """
        Calculate forward kinematics for 2-link planar arm
        theta_hip = 0 when thigh points downward (-90 degrees from horizontal)
        """
        # Convert angles to radians and adjust for coordinate system
        # Since theta_hip = 0 when thigh points downward (-90 degrees)
        theta1 = req.theta_hip - math.pi/2  # Adjust reference frame
        theta2 = req.theta_knee
        
        # Forward kinematics equations
        # Position of end effector
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        
        # Create response
        response = ForwardKinematicsLegResponse()
        response.x = x
        response.y = y
        
        rospy.loginfo(f"FK: theta_hip={{req.theta_hip:.3f}}, theta_knee={{req.theta_knee:.3f}} -> x={{x:.3f}}, y={{y:.3f}}")
        
        return response

def main():
    rospy.init_node('forward_kinematics_service')
    
    # Create service instance
    fk_service = ForwardKinematicsService()
    
    rospy.loginfo("Forward Kinematics Service started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
