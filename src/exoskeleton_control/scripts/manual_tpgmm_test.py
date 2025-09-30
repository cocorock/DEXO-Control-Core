#!/usr/bin/env python3

"""
Manual TPGMM Test Script
Interactive command-line tool to manually test the TPGMM trajectory generator.
"""

import rospy
from exoskeleton_control.msg import FSMState, EStopTrigger

class ManualTPGMMTest:
    def __init__(self):
        rospy.init_node('manual_tpgmm_test')
        
        # Publishers
        self.fsm_pub = rospy.Publisher('/test/fsm_state', FSMState, queue_size=1)
        self.estop_pub = rospy.Publisher('/test/e_stop_trigger', EStopTrigger, queue_size=1)
        
        # Available states
        self.available_states = ['INIT', 'CALIBRATION_PROCESS', 'READY', 'WALKING', 'STOPPING', 'E_STOP']
        
        rospy.loginfo("Manual TPGMM Test initialized")
        self.print_help()

    def print_help(self):
        """Print available commands"""
        print("\n" + "="*60)
        print("MANUAL TPGMM TRAJECTORY GENERATOR TEST")
        print("="*60)
        print("Available commands:")
        print("  State transitions:")
        for i, state in enumerate(self.available_states):
            print(f"    {i+1}: {state}")
        print("  Emergency stop:")
        print("    e: Trigger emergency stop")
        print("    r: Release emergency stop")
        print("  Other:")
        print("    h: Show this help")
        print("    q: Quit")
        print("="*60)

    def publish_fsm_state(self, state):
        """Publish FSM state"""
        msg = FSMState()
        msg.header.stamp = rospy.Time.now()
        msg.state = state
        self.fsm_pub.publish(msg)
        print(f"✓ Published FSM state: {state}")

    def publish_estop(self, trigger, state="MANUAL_TEST"):
        """Publish emergency stop"""
        msg = EStopTrigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = trigger
        msg.state = state
        self.estop_pub.publish(msg)
        action = "Triggered" if trigger else "Released"
        print(f"✓ {action} emergency stop")

    def run(self):
        """Main interactive loop"""
        try:
            while not rospy.is_shutdown():
                try:
                    user_input = input("\nEnter command (h for help): ").strip().lower()
                    
                    if user_input == 'q':
                        print("Exiting...")
                        break
                    elif user_input == 'h':
                        self.print_help()
                    elif user_input == 'e':
                        self.publish_estop(True)
                    elif user_input == 'r':
                        self.publish_estop(False)
                    elif user_input.isdigit():
                        state_idx = int(user_input) - 1
                        if 0 <= state_idx < len(self.available_states):
                            self.publish_fsm_state(self.available_states[state_idx])
                        else:
                            print(f"✗ Invalid state number. Use 1-{len(self.available_states)}")
                    else:
                        print("✗ Unknown command. Type 'h' for help.")
                        
                except KeyboardInterrupt:
                    print("\nExiting...")
                    break
                except EOFError:
                    print("\nExiting...")
                    break
                except Exception as e:
                    print(f"✗ Error: {e}")
                    
        except KeyboardInterrupt:
            pass
        
        print("Manual TPGMM test finished.")

if __name__ == '__main__':
    try:
        test = ManualTPGMMTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")