#!/usr/bin/env python

import rospy
import math
from exoskeleton_control.srv import InverseKinematicsLeg, InverseKinematicsLegResponse

class InverseKinematicsLegService:
    def __init__(self):
        # Arm parameters (adjust based on your robot)
        self.L1 = 0.39  # Length of thigh (first link) in meters
        self.L2 = 0.42  # Length of shin (second link) in meters
        
        # Joint limits (in radians) - adjust based on your robot
        self.theta1_min = -math.pi      # Hip joint minimum angle
        self.theta1_max =  math.pi       # Hip joint maximum angle
        self.theta2_min = -math.pi      # Knee joint minimum angle
        self.theta2_max = 0     #math.pi  # Knee joint maximum angle
        
        # Error codes
        self.ERROR_UNREACHABLE = -5.0   # Target position unreachable
        self.ERROR_JOINT_LIMITS = -6.0  # Joint angles outside valid range
        
        # Initialize the service
        self.service = rospy.Service('inverse_kinematics_leg', InverseKinematicsLeg, 
                                   self.calculate_inverse_kinematics)
        rospy.loginfo("Inverse Kinematics Leg Service Ready")
    
    def calculate_inverse_kinematics(self, req):
        """
        Calculate inverse kinematics for 2-link planar arm (leg)
        Origin at hip, theta_hip = 0 when thigh points downward
        """
        x = req.ankle_X_pos
        y = req.ankle_Y_pos
        
        # Calculate distance from hip to ankle
        distance = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if distance > (self.L1 + self.L2) or distance < abs(self.L1 - self.L2):
            rospy.logwarn(f"Target unreachable: distance={{distance:.3f}}, max_reach={{self.L1 + self.L2:.3f}}")
            response = InverseKinematicsLegResponse()
            response.theta1 = self.ERROR_UNREACHABLE
            response.theta2 = self.ERROR_UNREACHABLE
            return response
        
        # Calculate knee angle using law of cosines
        cos_theta2 = (distance*distance - self.L1*self.L1 - self.L2*self.L2) / (2 * self.L1 * self.L2)
        
        # Clamp cos_theta2 to valid range to handle numerical errors
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # Choose elbow-down configuration (negative knee angle) ==============<<<<
        theta2 = -math.acos(cos_theta2)
        
        # Calculate hip angle
        alpha = math.atan2(y, x)  # Angle from hip to target
        beta = math.acos((self.L1*self.L1 + distance*distance - self.L2*self.L2) / (2 * self.L1 * distance))
        
        theta1_raw = alpha - beta
        
        # Adjust for coordinate system (theta_hip = 0 when thigh points downward)
        theta1 = theta1_raw + math.pi/2
        
        # Normalize angles to [-pi, pi]
        theta1 = self.normalize_angle(theta1)
        theta2 = self.normalize_angle(theta2)
        
        # Check joint limits
        if (theta1 < self.theta1_min or theta1 > self.theta1_max or 
            theta2 < self.theta2_min or theta2 > self.theta2_max):
            rospy.logwarn(f"Joint limits exceeded: theta1={{theta1:.3f}} [{{self.theta1_min:.3f}}, {{self.theta1_max:.3f}}], "
                         f"theta2={{theta2:.3f}} [{{self.theta2_min:.3f}}, {{self.theta2_max:.3f}}]")
            response = InverseKinematicsLegResponse()
            response.theta1 = self.ERROR_JOINT_LIMITS
            response.theta2 = self.ERROR_JOINT_LIMITS
            return response
        
        # Create successful response
        response = InverseKinematicsLegResponse()
        response.theta1 = theta1
        response.theta2 = theta2
        
        rospy.loginfo(f"IK: target=({{x:.3f}}, {{y:.3f}}) -> theta1={{theta1:.3f}}, theta2={{theta2:.3f}}")
        
        return response
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main():
    rospy.init_node('inverse_kinematics_leg_service')
    
    # Create service instance
    ik_service = InverseKinematicsLegService()
    
    rospy.loginfo("Inverse Kinematics Leg Service started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
