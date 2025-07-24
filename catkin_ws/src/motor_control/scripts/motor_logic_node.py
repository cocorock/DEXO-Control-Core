#!/usr/bin/env python

import rospy
from motor_control.msg import MotorCommand, MotorState

class MotorLogicNode:
    def __init__(self):
        # The node is initialized with a unique name from the launch file
        rospy.init_node('motor_logic_node', anonymous=True)
        
        # Get parameters from the launch file
        self.motor_name = rospy.get_param('~motor_name', 'hip_right')
        self.timeout_seconds = rospy.get_param('~timeout', 0.5) # Default timeout of 0.5 seconds
        
        rospy.loginfo("Starting logic node for motor: {} with a timeout of {}s".format(self.motor_name, self.timeout_seconds))

        # This node publishes commands for its specific motor
        self.command_pub = rospy.Publisher('/{}/motor_command'.format(self.motor_name), MotorCommand, queue_size=10)
        
        # This node subscribes to the state of its specific motor
        rospy.Subscriber('/{}/motor_state'.format(self.motor_name), MotorState, self.state_callback)

        self.latest_state = None
        self.last_state_time = rospy.get_time() # Initialize with start time
        self.is_timed_out = False

    def state_callback(self, msg):
        """Stores the latest state of the motor and updates the timestamp."""
        self.latest_state = msg
        self.last_state_time = rospy.get_time()
        
        # If we were in a timed-out state, log that we have recovered.
        if self.is_timed_out:
            rospy.loginfo("Motor '{}' has recovered.".format(self.motor_name))
            self.is_timed_out = False

    def check_for_timeout(self):
        """Checks if the motor has timed out."""
        if rospy.get_time() - self.last_state_time > self.timeout_seconds:
            if not self.is_timed_out:
                rospy.logerr("Motor '{}' has timed out! No state received for over {} seconds.".format(self.motor_name, self.timeout_seconds))
                self.is_timed_out = True
                # In a real system, you would publish an emergency stop message here
                # or take other safety measures.
        return self.is_timed_out

    def run_logic(self):
        """This is where you would implement your actual control logic."""
        # IMPORTANT: Do not send commands if the motor has timed out.
        if self.is_timed_out:
            # Send a zero-torque command as a safety measure
            cmd = MotorCommand()
            cmd.p_in = 0.0
            cmd.v_in = 0.0
            cmd.kp_in = 0.0 
            cmd.kd_in = 0.0
            cmd.t_in = 0.0
            self.command_pub.publish(cmd)
            return

        # For this example, we'll just publish a command to hold position 0.
        cmd = MotorCommand()
        cmd.p_in = 0.0
        cmd.v_in = 0.0
        cmd.kp_in = 5.0  # A bit of stiffness
        cmd.kd_in = 1.0  # A bit of damping
        cmd.t_in = 0.0
        
        self.command_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            if self.check_for_timeout():
                # If timed out, we just want to send safe commands, which is handled
                # in run_logic().
                pass
            
            self.run_logic()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MotorLogicNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
