#!/usr/bin/env python3

"""
Bluetooth Smart Crutch Node
Receives data from ESP32-based smart crutch via Bluetooth RFCOMM
Parses compact format: timestamp,analog_command,force,qw,qx,qy,qz
Publishes CrutchState and CrutchSensorData topics
"""

import rospy
import serial
import struct
import time
import threading
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from exoskeleton_control.msg import CrutchState, CrutchSensorData


class BluetoothCrutchNode:
    def __init__(self):
        rospy.init_node('bluetooth_crutch_node', anonymous=True)
        
        # Parameters
        self.bluetooth_device = rospy.get_param('~bluetooth_device', '/dev/rfcomm0')
        self.baud_rate = rospy.get_param('~baud_rate', 921600)
        self.reconnect_interval = rospy.get_param('~reconnect_interval', 5.0)  # seconds
        self.publish_rate = rospy.get_param('~publish_rate', 20)  # Hz
        self.debug_output = rospy.get_param('~debug_output', True)
        
        # Publishers
        self.crutch_state_pub = rospy.Publisher('/crutch/state', CrutchState, queue_size=10)
        self.crutch_sensor_pub = rospy.Publisher('/crutch/sensor_data', CrutchSensorData, queue_size=10)
        
        # Serial connection
        self.serial_conn = None
        self.connection_lock = threading.Lock()
        
        # Data storage
        self.last_state = "S"  # Default to standing
        self.last_force = 0.0
        self.last_orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
        self.last_data_time = rospy.Time.now()
        
        # Statistics
        self.messages_received = 0
        self.parse_errors = 0
        self.connection_attempts = 0
        
        rospy.loginfo("Bluetooth Crutch Node initialized")
        rospy.loginfo(f"Device: {self.bluetooth_device}")
        rospy.loginfo(f"Baud rate: {self.baud_rate}")
        rospy.loginfo(f"Expected format: timestamp,analog_command,force,qw,qx,qy,qz")
        
    def connect_bluetooth(self):
        """Attempt to connect to Bluetooth device"""
        with self.connection_lock:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.close()
                    
                self.connection_attempts += 1
                rospy.loginfo(f"Attempting Bluetooth connection #{self.connection_attempts} to {self.bluetooth_device}")
                
                # Check if device exists before attempting connection
                if not self.check_device_exists():
                    rospy.logwarn(f"Device {self.bluetooth_device} does not exist. Run Bluetooth setup first.")
                    return False
                
                self.serial_conn = serial.Serial(
                    port=self.bluetooth_device,
                    baudrate=self.baud_rate,
                    timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                
                # Wait a moment for connection to stabilize
                time.sleep(0.5)
                
                if self.serial_conn.is_open:
                    rospy.loginfo(f"Successfully connected to {self.bluetooth_device}")
                    # Clear any buffered data
                    self.serial_conn.reset_input_buffer()
                    self.serial_conn.reset_output_buffer()
                    return True
                else:
                    rospy.logerr("Failed to open Bluetooth connection")
                    return False
                    
            except Exception as e:
                rospy.logerr(f"Bluetooth connection error: {e}")
                if "Permission denied" in str(e):
                    rospy.logerr("Permission denied. Try: sudo chmod 666 /dev/rfcomm0")
                elif "No such file" in str(e):
                    rospy.logerr("Device not found. Run Bluetooth setup script first.")
                
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                    self.serial_conn = None
                return False
    
    def check_device_exists(self):
        """Check if Bluetooth device file exists and has proper permissions"""
        import os
        import stat
        
        if not os.path.exists(self.bluetooth_device):
            return False
        
        # Check if we have read/write permissions
        try:
            mode = os.stat(self.bluetooth_device).st_mode
            if stat.S_ISCHR(mode):  # Character device
                return os.access(self.bluetooth_device, os.R_OK | os.W_OK)
        except OSError:
            return False
        
        return True
    
    def parse_compact_message(self, line):
        """
        Parse compact format message: timestamp,analog_command,force,qw,qx,qy,qz
        force is integer (actual * 100)
        quaternion values are integers (actual * 1000)
        """
        try:
            parts = line.strip().split(',')
            if len(parts) != 7:
                if self.debug_output and not line.startswith('PERF:') and not line.startswith('QUEUES:'):
                    rospy.logdebug(f"Invalid message format (expected 7 parts, got {len(parts)}): {line}")
                return None
                
            timestamp_ms = int(parts[0])
            analog_command = parts[1]
            force_int = int(parts[2])
            qw_int = int(parts[3])
            qx_int = int(parts[4])
            qy_int = int(parts[5])
            qz_int = int(parts[6])
            
            # Convert to actual values
            force = force_int / 100.0  # Convert from integer representation
            qw = qw_int / 1000.0      # Convert from integer representation
            qx = qx_int / 1000.0
            qy = qy_int / 1000.0
            qz = qz_int / 1000.0
            
            # Debug emergency stop parsing
            if analog_command == "ES":
                rospy.logwarn(f"🚨 PARSING ES STATE: line='{line}', cmd='{analog_command}'")
            
            return {
                'timestamp_ms': timestamp_ms,
                'state': analog_command,
                'force': force,
                'quaternion': (qw, qx, qy, qz)
            }
            
        except (ValueError, IndexError) as e:
            # Don't log errors for performance/debug messages
            if not line.startswith('PERF:') and not line.startswith('QUEUES:') and not line.startswith('DEBUG:'):
                self.parse_errors += 1
                if self.debug_output:
                    rospy.logdebug(f"Parse error: {e}, line: {line}")
            return None
    
    def publish_data(self, data):
        """Publish parsed data to ROS topics"""
        current_time = rospy.Time.now()
        
        # Create header
        header = Header()
        header.stamp = current_time
        header.frame_id = "crutch"
        
        # Publish state if changed OR if it's an emergency stop (always publish ES for safety)
        if data['state'] != self.last_state or data['state'] == "ES":
            state_msg = CrutchState()
            state_msg.header = header
            state_msg.state = data['state']
            self.crutch_state_pub.publish(state_msg)
            
            # Special handling for emergency stop
            if data['state'] == "ES":
                rospy.logwarn(f"🚨 EMERGENCY STOP ACTIVATED! State: {self.last_state} -> {data['state']}")
            else:
                rospy.loginfo(f"Crutch state changed: {self.last_state} -> {data['state']}")
            
            self.last_state = data['state']
        
        # Always publish sensor data
        sensor_msg = CrutchSensorData()
        sensor_msg.header = header
        sensor_msg.force = data['force']
        sensor_msg.orientation.w = data['quaternion'][0]
        sensor_msg.orientation.x = data['quaternion'][1]
        sensor_msg.orientation.y = data['quaternion'][2]
        sensor_msg.orientation.z = data['quaternion'][3]
        
        self.crutch_sensor_pub.publish(sensor_msg)
        
        # Update stored values
        self.last_force = data['force']
        self.last_orientation = sensor_msg.orientation
        self.last_data_time = current_time
        
        self.messages_received += 1
    
    def publish_emergency_state(self):
        """Force publish emergency stop state"""
        current_time = rospy.Time.now()
        
        # Create header
        header = Header()
        header.stamp = current_time
        header.frame_id = "crutch"
        
        # Publish emergency state
        state_msg = CrutchState()
        state_msg.header = header
        state_msg.state = "ES"
        self.crutch_state_pub.publish(state_msg)
        
        rospy.logwarn("🚨 EMERGENCY STOP PUBLISHED (from ESP32 message)")
        self.last_state = "ES"
        self.last_data_time = current_time
    
    def read_bluetooth_data(self):
        """Main data reading loop"""
        buffer = ""
        
        while not rospy.is_shutdown():
            with self.connection_lock:
                if not self.serial_conn or not self.serial_conn.is_open:
                    rospy.logwarn("Bluetooth connection lost, attempting reconnection...")
                    if not self.connect_bluetooth():
                        rospy.logwarn(f"Reconnection failed, retrying in {self.reconnect_interval} seconds")
                        rospy.sleep(self.reconnect_interval)
                        continue
                
                try:
                    # Read available data
                    if self.serial_conn.in_waiting > 0:
                        raw_data = self.serial_conn.read(self.serial_conn.in_waiting)
                        try:
                            data_str = raw_data.decode('utf-8', errors='ignore')
                            buffer += data_str
                        except UnicodeDecodeError:
                            rospy.logwarn("Unicode decode error, skipping malformed data")
                            continue
                        
                        # Process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            
                            if line:
                                # Handle different message types
                                if line.startswith('PERF:') or line.startswith('QUEUES:') or line.startswith('DEBUG:'):
                                    # Performance/status messages
                                    if self.debug_output:
                                        rospy.loginfo(f"ESP32: {line}")
                                elif line.startswith('EMERGENCY STOP'):
                                    # Handle ESP32 emergency stop messages
                                    rospy.logwarn(f"🚨 ESP32 EMERGENCY: {line}")
                                    # Force publish emergency state even if we haven't received data yet
                                    self.publish_emergency_state()
                                elif ',' in line and len(line.split(',')) == 7:
                                    # Compact format sensor data
                                    parsed_data = self.parse_compact_message(line)
                                    if parsed_data:
                                        self.publish_data(parsed_data)
                                        if self.debug_output and self.messages_received % 100 == 0:
                                            rospy.loginfo(f"Processed {self.messages_received} messages, "
                                                        f"{self.parse_errors} parse errors")
                                else:
                                    # Other ESP32 messages (calibration, state changes, etc.)
                                    rospy.loginfo(f"ESP32: {line}")
                    
                    # Small sleep to prevent CPU spinning
                    rospy.sleep(0.001)  # 1ms
                    
                except serial.SerialException as e:
                    rospy.logerr(f"Serial read error: {e}")
                    if self.serial_conn:
                        try:
                            self.serial_conn.close()
                        except:
                            pass
                        self.serial_conn = None
                    rospy.sleep(1.0)
                except Exception as e:
                    rospy.logerr(f"Unexpected error in read loop: {e}")
                    rospy.sleep(1.0)
    
    def print_statistics(self):
        """Print node statistics"""
        rate = rospy.Rate(0.1)  # Every 10 seconds
        
        while not rospy.is_shutdown():
            if self.messages_received > 0:
                time_since_last = (rospy.Time.now() - self.last_data_time).to_sec()
                connection_status = "Connected" if (self.serial_conn and self.serial_conn.is_open) else "Disconnected"
                
                rospy.loginfo(f"BT Crutch Stats - Messages: {self.messages_received}, "
                            f"Parse Errors: {self.parse_errors}, "
                            f"Connection: {connection_status}, "
                            f"Last Data: {time_since_last:.1f}s ago, "
                            f"State: {self.last_state}, "
                            f"Force: {self.last_force:.2f}g")
            
            rate.sleep()
    
    def run(self):
        """Main run method"""
        rospy.loginfo("Starting Bluetooth Crutch Node...")
        
        # Initial connection attempt
        if not self.connect_bluetooth():
            rospy.logwarn("Initial connection failed, will retry automatically")
        
        # Start statistics thread
        stats_thread = threading.Thread(target=self.print_statistics)
        stats_thread.daemon = True
        stats_thread.start()
        
        # Start main data reading loop
        try:
            self.read_bluetooth_data()
        except KeyboardInterrupt:
            rospy.loginfo("Shutdown requested")
        except Exception as e:
            rospy.logerr(f"Fatal error: {e}")
        finally:
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                    rospy.loginfo("Bluetooth connection closed")
                except:
                    pass


if __name__ == '__main__':
    try:
        node = BluetoothCrutchNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Bluetooth Crutch Node terminated")
    except Exception as e:
        rospy.logerr(f"Failed to start Bluetooth Crutch Node: {e}")