#!/usr/bin/env python3
"""
Dummy Crutches Node - Manual trigger interface for testing
Provides CLI interface to send stop_trigger and calibration_trigger messages
"""

import rospy
import threading
import sys
from exoskeleton_control.msg import CrutchCommand

class DummyCrutchesNode:
    def __init__(self):
        rospy.init_node('dummy_crutches_node')
        
        # Publishers
        self.crutch_command_pub = rospy.Publisher('/crutch_command', CrutchCommand, queue_size=1)
        
        # CLI interface running in separate thread
        self.cli_thread = threading.Thread(target=self.cli_interface)
        self.cli_thread.daemon = True
        
        rospy.loginfo("Dummy Crutches Node started")
        rospy.loginfo("Use the CLI interface to send commands:")
        rospy.loginfo("  'c' or 'calibrate' - Send ST_CALIBRATION_TRIG")
        rospy.loginfo("  'w' or 'walk' - Send ST_WALKING_TRIG")
        rospy.loginfo("  's' or 'stop' - Send STOP_TRIG")
        rospy.loginfo("  'x' or 'shutdown' - Send SHUTDOWN command")
        rospy.loginfo("  'h' or 'help' - Show help")
        rospy.loginfo("  'q' or 'quit' - Exit")

    def send_crutch_command(self, command, description):
        """Send crutch command message."""
        msg = CrutchCommand()
        msg.header.stamp = rospy.Time.now()
        msg.command = command
        
        self.crutch_command_pub.publish(msg)
        rospy.loginfo(f"✅ {description} sent")

    def send_calibration_trigger(self):
        """Send ST_CALIBRATION_TRIG command."""
        self.send_crutch_command(CrutchCommand.ST_CALIBRATION_TRIG, "ST_CALIBRATION_TRIG")

    def send_walking_trigger(self):
        """Send ST_WALKING_TRIG command."""
        self.send_crutch_command(CrutchCommand.ST_WALKING_TRIG, "ST_WALKING_TRIG")

    def send_stop_trigger(self):
        """Send STOP_TRIG command."""
        self.send_crutch_command(CrutchCommand.STOP_TRIG, "STOP_TRIG")

    def send_shutdown_command(self):
        """Send SHUTDOWN command."""
        self.send_crutch_command(CrutchCommand.SHUTDOWN, "SHUTDOWN")

    def show_help(self):
        """Show available commands."""
        print("\n" + "="*50)
        print("DUMMY CRUTCHES - MANUAL CONTROL INTERFACE")
        print("="*50)
        print("Available commands:")
        print("  c, calibrate  - Send ST_CALIBRATION_TRIG")
        print("  w, walk       - Send ST_WALKING_TRIG")
        print("  s, stop       - Send STOP_TRIG")
        print("  x, shutdown   - Send SHUTDOWN command")
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
        print(f"  crutch_command: {self.crutch_command_pub.get_num_connections()} subscribers")
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
                
                elif command in ['w', 'walk']:
                    self.send_walking_trigger()
                
                elif command in ['s', 'stop']:
                    self.send_stop_trigger()
                
                elif command in ['x', 'shutdown']:
                    self.send_shutdown_command()
                
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