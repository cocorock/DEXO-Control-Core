#!/usr/bin/env python3

"""
Test FSM State Publisher for TPGMM Trajectory Generator Testing
This node simulates system state transitions to test the TPGMM trajectory generator.
"""

import rospy
from exoskeleton_control.msg import FSMState

class TestFSMPublisher:
    def __init__(self):
        rospy.init_node('test_fsm_publisher')
        
        # Load parameters
        self.state_sequence = rospy.get_param('~state_sequence', ['INIT', 'READY', 'WALKING', 'STOPPING', 'READY'])
        self.state_duration = rospy.get_param('~state_duration', 5.0)  # seconds
        
        # Publisher
        self.fsm_pub = rospy.Publisher('fsm_state', FSMState, queue_size=1)
        
        # State tracking
        self.current_state_index = 0
        
        rospy.loginfo("Test FSM Publisher initialized")
        rospy.loginfo(f"State sequence: {self.state_sequence}")
        rospy.loginfo(f"State duration: {self.state_duration}s")

    def run(self):
        """Main execution loop"""
        rate = rospy.Rate(1.0)  # 1 Hz for state updates
        
        while not rospy.is_shutdown():
            try:
                # Get current state
                current_state = self.state_sequence[self.current_state_index]
                
                # Create and publish FSM state message
                msg = FSMState()
                msg.header.stamp = rospy.Time.now()
                msg.state = current_state
                
                self.fsm_pub.publish(msg)
                
                rospy.loginfo(f"Published FSM state: {current_state} (index: {self.current_state_index})")
                
                # Wait for state duration
                rospy.sleep(self.state_duration)
                
                # Move to next state
                self.current_state_index = (self.current_state_index + 1) % len(self.state_sequence)
                
            except KeyboardInterrupt:
                rospy.loginfo("Test FSM Publisher shutting down...")
                break
            except Exception as e:
                rospy.logerr(f"Error in test FSM publisher: {e}")
                rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TestFSMPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test FSM Publisher shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")