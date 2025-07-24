#!/usr/bin/env python

import rospy
import math
from exoskeleton_control.srv import InverseKinematicsClosedChain, InverseKinematicsClosedChainResponse

class InverseKinematicsClosedChainService:
    def __init__(self):
        # Initialize the service
        self.service = rospy.Service('inverse_kinematics_closed_chain', 
                                   InverseKinematicsClosedChain, 
                                   self.calculate_closed_chain_ik)
        rospy.loginfo("Inverse Kinematics Closed Chain Service Ready")
    
    def calculate_closed_chain_ik(self, req):
        """
        Calculate x_target using the formula:
        x_target = X_max + X_min + r*cos(theta_torso)
        """
        try:
            # Extract parameters from request
            X_max = req.X_max
            X_min = req.X_min
            theta_torso = req.theta_torso
            r = 0.1 # Default value for r, you can adjust this
            
            # Calculate x_target using the given formula
            x_target = X_max + X_min + r * math.cos(theta_torso)
            
            # Create response
            response = InverseKinematicsClosedChainResponse()
            response.X_target = x_target
            
            rospy.loginfo(f"Closed Chain IK: X_max={{X_max:.3f}}, X_min={{X_min:.3f}}, "
                         f"theta_torso={{theta_torso:.3f}} -> X_target={{x_target:.3f}}")
            
            return response
            
        except Exception as e:
            rospy.logerr(f"Error in closed chain IK calculation: {{e}}")
            response = InverseKinematicsClosedChainResponse()
            response.X_target = float('nan')  # Return NaN on error
            return response

def main():
    rospy.init_node('inverse_kinematics_closed_chain_service')
    
    # Create service instance
    ik_service = InverseKinematicsClosedChainService()
    
    rospy.loginfo("Inverse Kinematics Closed Chain Service started")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
