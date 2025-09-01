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
        
        # Parameters - Dual Crutch Support
        self.bluetooth_right_device = rospy.get_param('~bluetooth_right_device', '/dev/rfcomm0')
        self.bluetooth_left_device = rospy.get_param('~bluetooth_left_device', '/dev/rfcomm1')
        self.baud_rate = rospy.get_param('~baud_rate', 921600)
        self.reconnect_interval = rospy.get_param('~reconnect_interval', 5.0)  # seconds
        self.publish_rate = rospy.get_param('~publish_rate', 20)  # Hz
        self.debug_output = rospy.get_param('~debug_output', True)
        
        # Publishers
        self.crutch_state_pub = rospy.Publisher('/crutch/state', CrutchState, queue_size=10)
        self.crutch_sensor_pub = rospy.Publisher('/crutch/sensor_data', CrutchSensorData, queue_size=10)
        
        # Serial connections - Dual Crutch
        self.serial_right_conn = None
        self.serial_left_conn = None
        self.connection_right_lock = threading.Lock()
        self.connection_left_lock = threading.Lock()
        
        # Data storage - Dual Crutch
        self.last_state = "S"  # Default to standing (global state)
        self.last_right_force = 0.0
        self.last_left_force = 0.0
        self.last_right_orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
        self.last_left_orientation = Quaternion(0, 0, 0, 1)   # Identity quaternion
        self.last_data_time = rospy.Time.now()
        
        # Statistics - Dual Crutch
        self.right_messages_received = 0
        self.left_messages_received = 0
        self.right_parse_errors = 0
        self.left_parse_errors = 0
        self.right_connection_attempts = 0
        self.left_connection_attempts = 0
        
        rospy.loginfo("Bluetooth Crutch Node initialized")
        rospy.loginfo(f"Right crutch device: {self.bluetooth_right_device}")
        rospy.loginfo(f"Left crutch device: {self.bluetooth_left_device}")
        rospy.loginfo(f"Baud rate: {self.baud_rate}")
        rospy.loginfo(f"Expected format: timestamp,analog_command,force,qw,qx,qy,qz")
        rospy.loginfo(f"Right crutch (priority): frame_id='crutch_right'")
        rospy.loginfo(f"Left crutch: frame_id='crutch_left'")
        
    def connect_bluetooth_right(self):
        """Attempt to connect to right crutch Bluetooth device"""
        with self.connection_right_lock:
            try:
                if self.serial_right_conn and self.serial_right_conn.is_open:
                    self.serial_right_conn.close()
                    
                self.right_connection_attempts += 1
                rospy.loginfo(f"Attempting right crutch Bluetooth connection #{self.right_connection_attempts} to {self.bluetooth_right_device}")
                
                # Check if device exists before attempting connection
                if not self.check_device_exists(self.bluetooth_right_device):
                    rospy.logwarn(f"Right crutch device {self.bluetooth_right_device} does not exist. Run Bluetooth setup first.")
                    return False
                
                self.serial_right_conn = serial.Serial(
                    port=self.bluetooth_right_device,
                    baudrate=self.baud_rate,
                    timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                
                # Wait a moment for connection to stabilize
                time.sleep(0.5)
                
                if self.serial_right_conn.is_open:
                    rospy.loginfo(f"Successfully connected to right crutch {self.bluetooth_right_device}")
                    # Clear any buffered data
                    self.serial_right_conn.reset_input_buffer()
                    self.serial_right_conn.reset_output_buffer()
                    return True
                else:
                    rospy.logerr("Failed to open right crutch Bluetooth connection")
                    return False
                    
            except Exception as e:
                rospy.logerr(f"Right crutch Bluetooth connection error: {e}")
                if "Permission denied" in str(e):
                    rospy.logerr(f"Permission denied. Try: sudo chmod 666 {self.bluetooth_right_device}")
                elif "No such file" in str(e):
                    rospy.logerr("Right crutch device not found. Run Bluetooth setup script first.")
                
                if self.serial_right_conn:
                    try:
                        self.serial_right_conn.close()
                    except:
                        pass
                    self.serial_right_conn = None
                return False
    
    def connect_bluetooth_left(self):
        """Attempt to connect to left crutch Bluetooth device"""
        with self.connection_left_lock:
            try:
                if self.serial_left_conn and self.serial_left_conn.is_open:
                    self.serial_left_conn.close()
                    
                self.left_connection_attempts += 1
                rospy.loginfo(f"Attempting left crutch Bluetooth connection #{self.left_connection_attempts} to {self.bluetooth_left_device}")
                
                # Check if device exists before attempting connection
                if not self.check_device_exists(self.bluetooth_left_device):
                    rospy.logwarn(f"Left crutch device {self.bluetooth_left_device} does not exist. Run Bluetooth setup first.")
                    return False
                
                self.serial_left_conn = serial.Serial(
                    port=self.bluetooth_left_device,
                    baudrate=self.baud_rate,
                    timeout=1.0,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                
                # Wait a moment for connection to stabilize
                time.sleep(0.5)
                
                if self.serial_left_conn.is_open:
                    rospy.loginfo(f"Successfully connected to left crutch {self.bluetooth_left_device}")
                    # Clear any buffered data
                    self.serial_left_conn.reset_input_buffer()
                    self.serial_left_conn.reset_output_buffer()
                    return True
                else:
                    rospy.logerr("Failed to open left crutch Bluetooth connection")
                    return False
                    
            except Exception as e:
                rospy.logerr(f"Left crutch Bluetooth connection error: {e}")
                if "Permission denied" in str(e):
                    rospy.logerr(f"Permission denied. Try: sudo chmod 666 {self.bluetooth_left_device}")
                elif "No such file" in str(e):
                    rospy.logerr("Left crutch device not found. Run Bluetooth setup script first.")
                
                if self.serial_left_conn:
                    try:
                        self.serial_left_conn.close()
                    except:
                        pass
                    self.serial_left_conn = None
                return False
    
    def check_device_exists(self, device_path):
        """Check if Bluetooth device file exists and has proper permissions"""
        import os
        import stat
        
        if not os.path.exists(device_path):
            return False
        
        # Check if we have read/write permissions
        try:
            mode = os.stat(device_path).st_mode
            if stat.S_ISCHR(mode):  # Character device
                return os.access(device_path, os.R_OK | os.W_OK)
        except OSError:
            return False
        
        return True
    
    def parse_compact_message(self, line, is_right_crutch=True):
        """
        Parse compact format message: timestamp,analog_command,force,qw,qx,qy,qz
        force is integer (actual * 100)
        quaternion values are integers (actual * 1000)
        """
        crutch_name = "Right" if is_right_crutch else "Left"
        
        try:
            parts = line.strip().split(',')
            if len(parts) != 7:
                if self.debug_output and not line.startswith('PERF:') and not line.startswith('QUEUES:'):
                    rospy.logdebug(f"{crutch_name} crutch: Invalid message format (expected 7 parts, got {len(parts)}): {line}")
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
                rospy.logwarn(f"🚨 {crutch_name} crutch PARSING ES STATE: line='{line}', cmd='{analog_command}'")
            
            return {
                'timestamp_ms': timestamp_ms,
                'state': analog_command,
                'force': force,
                'quaternion': (qw, qx, qy, qz)
            }
            
        except (ValueError, IndexError) as e:
            # Don't log errors for performance/debug messages
            if not line.startswith('PERF:') and not line.startswith('QUEUES:') and not line.startswith('DEBUG:'):
                if is_right_crutch:
                    self.right_parse_errors += 1
                else:
                    self.left_parse_errors += 1
                    
                if self.debug_output:
                    rospy.logdebug(f"{crutch_name} crutch parse error: {e}, line: {line}")
            return None
    
    def publish_data(self, data, is_right_crutch=True):
        """Publish parsed data to ROS topics with crutch identification"""
        current_time = rospy.Time.now()
        
        # Determine frame_id and crutch identifier
        if is_right_crutch:
            frame_id = "crutch_right"
            crutch_name = "Right"
        else:
            frame_id = "crutch_left"
            crutch_name = "Left"
        
        # Create header
        header = Header()
        header.stamp = current_time
        header.frame_id = frame_id
        
        # State management with priority logic (right crutch has priority)
        new_state = data['state']
        state_changed = False
        
        # Priority logic: Right crutch commands override left crutch commands
        if is_right_crutch:
            # Right crutch always updates global state
            if new_state != self.last_state or new_state == "ES":
                state_changed = True
        else:
            # Left crutch only updates state if right crutch is not commanding
            # For simplicity, we'll let left crutch update if different and not ES
            # ES from any crutch should always be processed
            if new_state == "ES" or (new_state != self.last_state and self.last_state != "ES"):
                state_changed = True
        
        # Publish state if changed
        if state_changed:
            state_msg = CrutchState()
            state_msg.header = header  # Frame ID indicates which crutch sent the command
            state_msg.state = new_state
            self.crutch_state_pub.publish(state_msg)
            
            # Special handling for emergency stop
            if new_state == "ES":
                rospy.logwarn(f"🚨 EMERGENCY STOP ACTIVATED by {crutch_name} crutch! State: {self.last_state} -> {new_state}")
            else:
                rospy.loginfo(f"{crutch_name} crutch state changed: {self.last_state} -> {new_state}")
            
            self.last_state = new_state
        
        # Always publish sensor data with appropriate frame_id
        sensor_msg = CrutchSensorData()
        sensor_msg.header = header
        sensor_msg.force = data['force']
        sensor_msg.orientation.w = data['quaternion'][0]
        sensor_msg.orientation.x = data['quaternion'][1]
        sensor_msg.orientation.y = data['quaternion'][2]
        sensor_msg.orientation.z = data['quaternion'][3]
        
        self.crutch_sensor_pub.publish(sensor_msg)
        
        # Update stored values per crutch
        if is_right_crutch:
            self.last_right_force = data['force']
            self.last_right_orientation = sensor_msg.orientation
            self.right_messages_received += 1
        else:
            self.last_left_force = data['force']
            self.last_left_orientation = sensor_msg.orientation
            self.left_messages_received += 1
        
        self.last_data_time = current_time
    
    def publish_emergency_state(self, is_right_crutch=True):
        """Force publish emergency stop state"""
        current_time = rospy.Time.now()
        crutch_name = "Right" if is_right_crutch else "Left"
        frame_id = "crutch_right" if is_right_crutch else "crutch_left"
        
        # Create header
        header = Header()
        header.stamp = current_time
        header.frame_id = frame_id
        
        # Publish emergency state
        state_msg = CrutchState()
        state_msg.header = header
        state_msg.state = "ES"
        self.crutch_state_pub.publish(state_msg)
        
        rospy.logwarn(f"🚨 EMERGENCY STOP PUBLISHED by {crutch_name} crutch (from ESP32 message)")
        self.last_state = "ES"
        self.last_data_time = current_time
    
    def read_bluetooth_data_crutch(self, is_right_crutch=True):
        """Data reading loop for a specific crutch"""
        crutch_name = "Right" if is_right_crutch else "Left"
        serial_conn = self.serial_right_conn if is_right_crutch else self.serial_left_conn
        connection_lock = self.connection_right_lock if is_right_crutch else self.connection_left_lock
        connect_func = self.connect_bluetooth_right if is_right_crutch else self.connect_bluetooth_left
        
        buffer = ""
        
        while not rospy.is_shutdown():
            with connection_lock:
                serial_conn = self.serial_right_conn if is_right_crutch else self.serial_left_conn
                
                if not serial_conn or not serial_conn.is_open:
                    rospy.logwarn(f"{crutch_name} crutch Bluetooth connection lost, attempting reconnection...")
                    if not connect_func():
                        rospy.logwarn(f"{crutch_name} crutch reconnection failed, retrying in {self.reconnect_interval} seconds")
                        rospy.sleep(self.reconnect_interval)
                        continue
                    serial_conn = self.serial_right_conn if is_right_crutch else self.serial_left_conn
                
                try:
                    # Read available data
                    if serial_conn.in_waiting > 0:
                        raw_data = serial_conn.read(serial_conn.in_waiting)
                        try:
                            data_str = raw_data.decode('utf-8', errors='ignore')
                            buffer += data_str
                        except UnicodeDecodeError:
                            rospy.logwarn(f"{crutch_name} crutch: Unicode decode error, skipping malformed data")
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
                                        rospy.loginfo(f"{crutch_name} ESP32: {line}")
                                elif line.startswith('EMERGENCY STOP'):
                                    # Handle ESP32 emergency stop messages
                                    rospy.logwarn(f"🚨 {crutch_name} ESP32 EMERGENCY: {line}")
                                    # Force publish emergency state even if we haven't received data yet
                                    self.publish_emergency_state(is_right_crutch)
                                elif ',' in line and len(line.split(',')) == 7:
                                    # Compact format sensor data
                                    parsed_data = self.parse_compact_message(line, is_right_crutch)
                                    if parsed_data:
                                        self.publish_data(parsed_data, is_right_crutch)
                                        
                                        # Debug output for message counts
                                        messages_count = self.right_messages_received if is_right_crutch else self.left_messages_received
                                        parse_errors = self.right_parse_errors if is_right_crutch else self.left_parse_errors
                                        
                                        if self.debug_output and messages_count % 100 == 0:
                                            rospy.loginfo(f"{crutch_name} crutch: Processed {messages_count} messages, "
                                                        f"{parse_errors} parse errors")
                                else:
                                    # Other ESP32 messages (calibration, state changes, etc.)
                                    rospy.loginfo(f"{crutch_name} ESP32: {line}")
                    
                    # Small sleep to prevent CPU spinning
                    rospy.sleep(0.001)  # 1ms
                    
                except serial.SerialException as e:
                    rospy.logerr(f"{crutch_name} crutch serial read error: {e}")
                    if serial_conn:
                        try:
                            serial_conn.close()
                        except:
                            pass
                        if is_right_crutch:
                            self.serial_right_conn = None
                        else:
                            self.serial_left_conn = None
                    rospy.sleep(1.0)
                except Exception as e:
                    rospy.logerr(f"{crutch_name} crutch unexpected error in read loop: {e}")
                    rospy.sleep(1.0)
    
    def print_statistics(self):
        """Print node statistics for dual crutch system"""
        rate = rospy.Rate(0.1)  # Every 10 seconds
        
        while not rospy.is_shutdown():
            total_messages = self.right_messages_received + self.left_messages_received
            if total_messages > 0:
                time_since_last = (rospy.Time.now() - self.last_data_time).to_sec()
                
                # Check connection status for both crutches
                right_connected = (self.serial_right_conn and self.serial_right_conn.is_open)
                left_connected = (self.serial_left_conn and self.serial_left_conn.is_open)
                connection_status = f"Right:{right_connected} Left:{left_connected}"
                
                rospy.loginfo(f"Dual BT Crutch Stats - "
                            f"Right Msgs: {self.right_messages_received}, "
                            f"Left Msgs: {self.left_messages_received}, "
                            f"Right Errors: {self.right_parse_errors}, "
                            f"Left Errors: {self.left_parse_errors}, "
                            f"Connections: {connection_status}, "
                            f"Last Data: {time_since_last:.1f}s ago, "
                            f"State: {self.last_state}, "
                            f"Right Force: {self.last_right_force:.2f}g, "
                            f"Left Force: {self.last_left_force:.2f}g")
            
            rate.sleep()
    
    def run(self):
        """Main run method for dual crutch system"""
        rospy.loginfo("Starting Dual Bluetooth Crutch Node...")
        
        # Initial connection attempts
        right_connected = self.connect_bluetooth_right()
        left_connected = self.connect_bluetooth_left()
        
        if not right_connected and not left_connected:
            rospy.logwarn("Both initial connections failed, will retry automatically")
        elif not right_connected:
            rospy.logwarn("Right crutch initial connection failed, will retry automatically")
        elif not left_connected:
            rospy.logwarn("Left crutch initial connection failed, will retry automatically")
        else:
            rospy.loginfo("Both crutches connected successfully!")
        
        # Start statistics thread
        stats_thread = threading.Thread(target=self.print_statistics)
        stats_thread.daemon = True
        stats_thread.start()
        
        # Start data reading threads for both crutches
        right_thread = threading.Thread(target=self.read_bluetooth_data_crutch, args=(True,))
        left_thread = threading.Thread(target=self.read_bluetooth_data_crutch, args=(False,))
        
        right_thread.daemon = True
        left_thread.daemon = True
        
        right_thread.start()
        left_thread.start()
        
        rospy.loginfo("Dual crutch data reading threads started")
        
        # Main thread waits for shutdown
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutdown requested")
        except Exception as e:
            rospy.logerr(f"Fatal error: {e}")
        finally:
            # Close both connections
            if self.serial_right_conn:
                try:
                    self.serial_right_conn.close()
                    rospy.loginfo("Right crutch Bluetooth connection closed")
                except:
                    pass
            if self.serial_left_conn:
                try:
                    self.serial_left_conn.close()
                    rospy.loginfo("Left crutch Bluetooth connection closed")
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