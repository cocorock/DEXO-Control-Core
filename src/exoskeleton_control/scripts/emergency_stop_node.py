#!/usr/bin/env python

import rospy
import smach
import smach_ros
from exoskeleton_control.msg import MotorStatus, EStopTrigger

# Define states
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibrating'])

    def execute(self, userdata):
        rospy.loginfo('Executing state INIT')
        return 'calibrating'

class Calibrating(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CALIBRATING')
        # Your calibration logic here
        return 'ready'

class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stopped', 'error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state READY')
        return 'stopped'

class Stopped(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STOPPED')
        return 'ready'

class Error(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ERROR')
        return 'init'

class EmergencyStopNode:
    def __init__(self):
        rospy.init_node('emergency_stop_node')

        # Subscribers
        rospy.Subscriber('MotorStatus', MotorStatus, self.motor_status_callback)

        # Publishers
        self.e_stop_trigger_pub = rospy.Publisher('e_stop_trigger', EStopTrigger, queue_size=10)

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['shutdown'])
        with self.sm:
            smach.StateMachine.add('INIT', Init(), transitions={'calibrating':'CALIBRATING'})
            smach.StateMachine.add('CALIBRATING', Calibrating(), transitions={'ready':'READY', 'error':'ERROR'})
            smach.StateMachine.add('READY', Ready(), transitions={'stopped':'STOPPED', 'error':'ERROR'})
            smach.StateMachine.add('STOPPED', Stopped(), transitions={'ready':'READY', 'error':'ERROR'})
            smach.StateMachine.add('ERROR', Error(), transitions={'init':'INIT'})

        # Create and start the introspection server for visualization
        sis = smach_ros.IntrospectionServer('smach_server', self.sm, '/SM_ROOT')
        sis.start()

    def motor_status_callback(self, msg):
        # Check for motor errors
        if msg.error_flags != 0:
            e_stop_msg = EStopTrigger()
            e_stop_msg.trigger = True
            e_stop_msg.state = self.sm.get_active_states()[0]
            self.e_stop_trigger_pub.publish(e_stop_msg)

    def run(self):
        self.sm.execute()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EmergencyStopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
