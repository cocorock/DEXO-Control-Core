#!/usr/bin/env python3
"""
Dummy Crutches Node - Manual trigger interface for testing
Provides CLI interface to send stop_trigger and calibration_trigger messages
"""

import rospy
import threading
import sys
from exoskeleton_control.msg import StopTrigger, CalibrationTrigger

class DummyCrutchesNode:
    def __init__(self):
        rospy.init_node('dummy_crutches_node')
        
        # Publishers
        self.stop_trigger_pub = rospy.Publisher('stop_trigger', StopTrigger, queue_size=1)
        self.calibration_trigger_pub = rospy.Publisher('manual_calibration_trigger', CalibrationTrigger, queue_size=1)
        
        # CLI interface running in separate thread
        self.cli_thread = threading.Thread(target=self.cli_interface)
        self.cli_thread.daemon = True
        
        rospy.loginfo("Dummy Crutches Node started")
        rospy.loginfo("Use the CLI interface to send triggers:")
        rospy.loginfo("  'c' or 'calibrate' - Send calibration trigger")
        rospy.loginfo("  's' or 'stop' - Send stop trigger")
        rospy.loginfo("  'r' or 'resume' - Clear stop trigger")
        rospy.loginfo("  'h' or 'help' - Show help")
        rospy.loginfo("  'q' or 'quit' - Exit")

    def send_calibration_trigger(self):
        """Send calibration trigger message."""
        msg = CalibrationTrigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = True
        
        self.calibration_trigger_pub.publish(msg)
        rospy.loginfo("✅ Calibration trigger sent")

    def send_stop_trigger(self, stop=True):
        """Send stop trigger message."""
        msg = StopTrigger()
        msg.header.stamp = rospy.Time.now()
        msg.trigger = stop
        
        self.stop_trigger_pub.publish(msg)
        
        if stop:
            rospy.logwarn("🛑 Stop trigger sent")
        else:
            rospy.loginfo("▶️  Stop trigger cleared (resume)")

    def show_help(self):
        """Show available commands."""
        print("\n" + "="*50)
        print("DUMMY CRUTCHES - MANUAL CONTROL INTERFACE")
        print("="*50)
        print("Available commands:")
        print("  c, calibrate  - Send calibration trigger")
        print("  s, stop       - Send emergency stop trigger")
        print("  r, resume     - Clear stop trigger (resume operation)")
        print("  h, help       - Show this help message")
        print("  q, quit       - Exit the program")
        print("  status        - Show current system status")
        print("="*50 + "\n")

    def show_status(self):
        """Show current system status."""
        print("\n" + "-"*30)
        print("SYSTEM STATUS")
        print("-"*30)
        print(f"Node: dummy_crutches_node")
        print(f"Publishers:")
        print(f"  stop_trigger: {self.stop_trigger_pub.get_num_connections()} subscribers")
        print(f"  calibration_trigger: {self.calibration_trigger_pub.get_num_connections()} subscribers")
        print(f"ROS Time: {rospy.Time.now()}")
        print("-"*30 + "\n")

    def cli_interface(self):
        """Command line interface for manual control."""
        print("\nDummy Crutches CLI Interface Ready")
        print("Type 'help' for available commands")
        
        while not rospy.is_shutdown():
            try:
                # Get user input
                command = input("\nEnter command: ").strip().lower()
                
                if command in ['c', 'calibrate']:
                    self.send_calibration_trigger()
                
                elif command in ['s', 'stop']:
                    self.send_stop_trigger(stop=True)
                
                elif command in ['r', 'resume']:
                    self.send_stop_trigger(stop=False)
                
                elif command in ['h', 'help']:
                    self.show_help()
                
                elif command == 'status':
                    self.show_status()
                
                elif command in ['q', 'quit', 'exit']:
                    rospy.loginfo("Exiting Dummy Crutches Node...")
                    rospy.signal_shutdown("User requested shutdown")
                    break
                
                elif command == '':
                    continue  # Empty input, just continue
                
                else:
                    print(f"Unknown command: '{command}'. Type 'help' for available commands.")
            
            except KeyboardInterrupt:
                print("\nKeyboard interrupt received. Exiting...")
                rospy.signal_shutdown("Keyboard interrupt")
                break
            
            except EOFError:
                print("\nEOF received. Exiting...")
                rospy.signal_shutdown("EOF received")
                break
            
            except Exception as e:
                rospy.logerr(f"Error in CLI interface: {e}")

    def run(self):
        """Main execution method."""
        rospy.loginfo("Starting Dummy Crutches Node CLI interface...")
        
        # Start CLI interface thread
        self.cli_thread.start()
        
        # Keep the main thread alive
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Dummy Crutches Node...")

if __name__ == '__main__':
    try:
        node = DummyCrutchesNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Dummy Crutches Node shutdown")
    except Exception as e:
        rospy.logerr(f"Unexpected error in Dummy Crutches Node: {e}")
        sys.exit(1)