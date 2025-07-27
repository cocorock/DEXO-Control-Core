#!/usr/bin/env python3
"""
Simple test script for MIT motor communication
Tests basic commands: Enter Mode, Zero Position, Exit Mode
"""

import can
import time
import sys

def send_can_message(bus, motor_id, data, description=""):
    """Send CAN message and print details"""
    try:
        print(f"\n--- Sending: {description} ---")
        print(f"Motor ID: 0x{motor_id:02X}")
        print(f"Data: {[hex(b) for b in data]}")
        
        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
        bus.send(message)
        print("✓ Message sent successfully")
        return True
    except can.CanError as e:
        print(f"✗ Failed to send message: {e}")
        return False

def read_responses(bus, timeout_sec=1.0, max_messages=5):
    """Read and display CAN responses"""
    print(f"\n--- Reading responses (timeout: {timeout_sec}s) ---")
    
    messages_received = 0
    start_time = time.time()
    
    while (time.time() - start_time) < timeout_sec and messages_received < max_messages:
        try:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
                
            messages_received += 1
            print(f"Response {messages_received}:")
            print(f"  ID: 0x{msg.arbitration_id:02X}")
            print(f"  Data: {[hex(b) for b in msg.data]}")
            print(f"  Extended: {msg.is_extended_id}")
            
            # Try to interpret if it looks like motor data
            if len(msg.data) >= 6:
                # Following the corrected format from PDF
                motor_id = msg.data[0]
                p_int = (msg.data[1] << 8) | msg.data[2] 
                v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
                i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]
                
                print(f"  Interpreted: Motor_ID={motor_id}, Pos_raw={p_int}, Vel_raw={v_int}, Current_raw={i_int}")
            
        except can.CanError as e:
            print(f"  Error reading: {e}")
            break
    
    if messages_received == 0:
        print("  No responses received")
    
    return messages_received

def test_motor_basic_commands(bus, motor_id):
    """Test the three basic MIT motor commands"""
    
    print(f"\n{'='*60}")
    print(f"TESTING MOTOR COMMUNICATION - Motor ID: 0x{motor_id:02X}")
    print(f"{'='*60}")
    
    # Command definitions from PDF
    enter_mode_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    zero_position_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
    exit_mode_cmd = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    
    success_count = 0
    
    # Test 1: Enter Motor Control Mode
    print(f"\nTEST 1: Enter Motor Control Mode")
    if send_can_message(bus, motor_id, enter_mode_cmd, "Enter Motor Control Mode"):
        success_count += 1
        responses = read_responses(bus, timeout_sec=2.0)
        if responses > 0:
            print(f"✓ Received {responses} response(s)")
        else:
            print("⚠ No responses received")
    
    time.sleep(0.5)  # Brief pause between commands
    
    # Test 2: Set Zero Position
    print(f"\nTEST 2: Set Current Position as Zero")
    if send_can_message(bus, motor_id, zero_position_cmd, "Set Zero Position"):
        success_count += 1
        responses = read_responses(bus, timeout_sec=2.0)
        if responses > 0:
            print(f"✓ Received {responses} response(s)")
        else:
            print("⚠ No responses received")
    
    time.sleep(0.5)  # Brief pause between commands
    
    # Test 3: Exit Motor Control Mode  
    print(f"\nTEST 3: Exit Motor Control Mode")
    if send_can_message(bus, motor_id, exit_mode_cmd, "Exit Motor Control Mode"):
        success_count += 1
        responses = read_responses(bus, timeout_sec=2.0)
        if responses > 0:
            print(f"✓ Received {responses} response(s)")
        else:
            print("⚠ No responses received")
    
    # Summary
    print(f"\n{'='*60}")
    print(f"TEST SUMMARY")
    print(f"{'='*60}")
    print(f"Commands sent successfully: {success_count}/3")
    
    if success_count == 3:
        print("✓ All commands sent successfully!")
        print("✓ Motor appears to be responding to CAN communication")
    else:
        print("⚠ Some commands failed to send")
        print("⚠ Check CAN interface and motor connection")

def main():
    print("MIT Motor Communication Test")
    print("=" * 40)
    
    # Get parameters from user
    try:
        bustype = input("Enter CAN bustype (e.g., 'socketcan', 'pcan'): ").strip() or "socketcan"
        channel = input("Enter CAN channel (e.g., 'can0', 'pcanusb1'): ").strip() or "can0"
        bitrate = int(input("Enter bitrate (default 1000000): ").strip() or "1000000")
        motor_id = int(input("Enter motor CAN ID (default 8): ").strip() or "8")
        
        if motor_id <= 0 or motor_id > 127:
            print("Warning: Motor ID should typically be 1-127")
            
    except ValueError:
        print("Invalid input! Using defaults.")
        bustype, channel, bitrate, motor_id = "socketcan", "can0", 1000000, 8
    
    bus = None
    
    try:
        # Initialize CAN interface
        print(f"\nInitializing CAN interface...")
        print(f"  Bustype: {bustype}")
        print(f"  Channel: {channel}")
        print(f"  Bitrate: {bitrate}")
        
        bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        print("✓ CAN interface initialized successfully!")
        
        # Run the test
        test_motor_basic_commands(bus, motor_id)
        
    except Exception as e:
        print(f"✗ Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check that CAN interface is available (e.g., 'ip link show can0')")
        print("2. Ensure proper permissions for CAN access")
        print("3. Verify motor is powered and connected")
        print("4. Check CAN bitrate matches motor configuration")
        
    finally:
        if bus:
            try:
                bus.shutdown()
                print("\n✓ CAN interface closed")
            except:
                pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)