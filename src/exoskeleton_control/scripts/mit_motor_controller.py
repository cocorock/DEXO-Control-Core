import can
import struct
import time
from enum import Enum

class MotorModel(Enum):
    AK10_9 = "AK10-9"
    AK60_6 = "AK60-6"
    AK70_10 = "AK70-10"
    AK80_6 = "AK80-6"
    AK80_9 = "AK80-9"
    AK80_64 = "AK80-64"
    AK80_8 = "AK80-8"

# Motor specifications based on Parameter Ranges table
MOTOR_SPECS = {
    MotorModel.AK10_9: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-50.0, 50.0),
        'torque_range': (-65.0, 65.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    },
    MotorModel.AK60_6: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-45.0, 45.0),
        'torque_range': (-15.0, 15.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    },
    MotorModel.AK70_10: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-50.0, 50.0),
        'torque_range': (-25.0, 25.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    },
    MotorModel.AK80_6: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-76.0, 76.0),
        'torque_range': (-12.0, 12.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    },
    MotorModel.AK80_9: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-50.0, 50.0),
        'torque_range': (-18.0, 18.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    },
    MotorModel.AK80_64: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-8.0, 8.0),
        'torque_range': (-144.0, 144.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    },
    MotorModel.AK80_8: {
        'position_range': (-12.5, 12.5),
        'speed_range': (-37.5, 37.5),
        'torque_range': (-32.0, 32.0),
        'kp_range': (0, 500),
        'kd_range': (0, 5)
    }
}

class MotorController:
    def __init__(self, model: MotorModel, controller_id: int = 0x8):
        """
        Initialize motor controller for specific model
        
        Args:
            model: MotorModel enum specifying the motor type
            controller_id: CAN ID for the motor controller (default: 0x8)
        """
        self.model = model
        self.controller_id = controller_id
        
        # Get motor specifications
        specs = MOTOR_SPECS[model]
        self.p_min, self.p_max = specs['position_range']
        self.v_min, self.v_max = specs['speed_range']
        self.t_min, self.t_max = specs['torque_range']
        self.kp_min, self.kp_max = specs['kp_range']
        self.kd_min, self.kd_max = specs['kd_range']
        
        print(f"Initialized {model.value} motor controller")
        print(f"Position range: {self.p_min} to {self.p_max} rad")
        print(f"Speed range: {self.v_min} to {self.v_max} rad/s")
        print(f"Torque range: {self.t_min} to {self.t_max} N·m")

class MotorState:
    def __init__(self):
        # Input commands
        self.p_in = 0.0      # Position command
        self.v_in = 0.0      # Velocity command
        self.kp_in = 0.0     # Position gain
        self.kd_in = 0.50    # Velocity gain
        self.t_in = 0.0      # Torque feedforward

        # Measured values from motor feedback
        self.motor_id = 0         # Motor ID from response
        self.p_out = 0.0         # Measured position
        self.v_out = 0.0         # Measured velocity
        self.t_out = 0.0         # Measured torque/current
        self.temperature = 0.0    # Motor temperature
        self.error_flag = 0       # Error status

def float_to_uint(x, x_min, x_max, bits):
    """Convert float to unsigned integer for CAN transmission"""
    span = x_max - x_min
    offset = x_min
    
    # Clamp value to range
    x = max(min(x, x_max), x_min)
    
    if bits == 12:
        return int((x - offset) * 4095.0 / span)
    elif bits == 16:
        return int((x - offset) * 65535.0 / span)
    return 0

def uint_to_float(x_int, x_min, x_max, bits):
    """Convert unsigned integer to float for motor feedback"""
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return (float(x_int) * span / 4095.0) + offset
    elif bits == 16:
        return (float(x_int) * span / 65535.0) + offset
    return 0.0

def send_can_message(bus, id, data):
    """Send CAN message with error handling"""
    try:
        if isinstance(data, bytearray):
            data = bytes(data)
        
        print(f"Sending CAN message: ID={hex(id)}, Data={[hex(b) for b in data]}")
        message = can.Message(arbitration_id=id, data=data, is_extended_id=False)
        bus.send(message)
        return True
    except can.CanError as e:
        print(f"Message NOT sent: {e}")
        return False

def enter_mode(bus, controller_id):
    """Enter MIT control mode"""
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    return send_can_message(bus, controller_id, data)

def exit_mode(bus, controller_id):
    """Exit MIT control mode"""
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    return send_can_message(bus, controller_id, data)

def zero_position(bus, controller_id):
    """Set current position as zero position"""
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
    return send_can_message(bus, controller_id, data)

def pack_cmd(bus, motor_controller: MotorController, motor_state: MotorState):
    """Pack and send control command to motor"""
    # Constrain values to motor limits
    p_des = max(min(motor_state.p_in, motor_controller.p_max), motor_controller.p_min)
    v_des = max(min(motor_state.v_in, motor_controller.v_max), motor_controller.v_min)
    kp = max(min(motor_state.kp_in, motor_controller.kp_max), motor_controller.kp_min)
    kd = max(min(motor_state.kd_in, motor_controller.kd_max), motor_controller.kd_min)
    t_ff = max(min(motor_state.t_in, motor_controller.t_max), motor_controller.t_min)

    # Convert to integers for CAN transmission
    p_int = float_to_uint(p_des, motor_controller.p_min, motor_controller.p_max, 16)
    v_int = float_to_uint(v_des, motor_controller.v_min, motor_controller.v_max, 12)
    kp_int = float_to_uint(kp, motor_controller.kp_min, motor_controller.kp_max, 12)
    kd_int = float_to_uint(kd, motor_controller.kd_min, motor_controller.kd_max, 12)
    t_int = float_to_uint(t_ff, motor_controller.t_min, motor_controller.t_max, 12)

    # Pack into 8-byte CAN frame according to protocol
    buf = bytearray(8)
    buf[0] = p_int >> 8                                    # Position High 8 bits
    buf[1] = p_int & 0xFF                                  # Position Low 8 bits
    buf[2] = v_int >> 4                                    # Speed High 8 bits
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)         # Speed Low 4 bits + KP High 4 bits
    buf[4] = kp_int & 0xFF                                 # KP Low 8 bits
    buf[5] = kd_int >> 4                                   # KD High 8 bits
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)         # KD Low 4 bits + Torque High 4 bits
    buf[7] = t_int & 0xFF                                  # Torque Low 8 bits

    return send_can_message(bus, motor_controller.controller_id, bytes(buf))

def is_command_echo(data):
    """Check if received data is a command echo rather than motor status"""
    if len(data) == 8:
        # Check for MIT mode commands
        if data[0] == 0xFF and data[1] == 0xFF and data[7] in [0xFC, 0xFD, 0xFE]:
            return True
        # Check if it looks like a position command (first byte is not motor ID)
        if data[0] != 0x17:  # 0x17 is typical motor ID in status responses
            return True
    return False

def unpack_reply(data, motor_controller: MotorController, motor_state: MotorState):
    """
    Unpack motor response data according to MIT Mode Driver Board Receive Data Definition
    
    Data format (8 bytes):
    DATA[0]: Motor ID (7-0)
    DATA[1]: Motor Position High 8 bits (7-0)
    DATA[2]: Motor Position Low 8 bits (7-0)
    DATA[3]: Motor Speed High 8 bits (7-0)
    DATA[4]: Motor Speed Low 4 bits (7-4) + KP Value High 4 bits (3-0)
    DATA[5]: KP Value Low 8 bits (7-0)
    DATA[6]: KD Value High 8 bits (7-0)
    DATA[7]: KD Value Low 4 bits (7-4) + Current Value High 4 bits (3-0)
    DATA[8]: Current Value Low 8 bits (7-0) - Wait, this should be in a different format
    
    Actually, based on the receive table, the format is:
    DATA[0]: Motor ID
    DATA[1-2]: Motor Position (16 bits)
    DATA[3]: Motor Speed High 8 bits
    DATA[4]: Motor Speed Low 4 bits + KP High 4 bits  
    DATA[5]: KP Low 8 bits
    DATA[6]: KD High 8 bits
    DATA[7]: KD Low 4 bits + Current High 4 bits
    DATA[8]: Current Low 8 bits
    
    But CAN frames are 8 bytes max, so let me check the actual receive format...
    """
    debugFlag = False

    if data is None or len(data) < 6:
        print("Note:len(data) < 6")
        return False

    # Skip command echoes
    if is_command_echo(data):
        print("Note: Received command echo, not motor status")
        return False

    # Extract motor ID
    motor_state.motor_id = data[0]
    
    # Extract position (16 bits)
    p_int = (data[1] << 8) | data[2]
    motor_state.p_out = uint_to_float(p_int, motor_controller.p_min, motor_controller.p_max, 16)
    
    # Extract velocity (12 bits)
    v_int = (data[3] << 4) | (data[4] >> 4)
    motor_state.v_out = uint_to_float(v_int, motor_controller.v_min, motor_controller.v_max, 12)
    
    # Extract current/torque (12 bits) - from lower 4 bits of data[4] and data[5]
    i_int = ((data[4] & 0xF) << 8) | data[5]
    motor_state.t_out = uint_to_float(i_int, motor_controller.t_min, motor_controller.t_max, 12)
    
    # If more data is available, extract additional fields
    if len(data) >= 7:
        # Could include temperature or error flags in extended responses
        motor_state.temperature = data[6] if len(data) > 6 else 0
        motor_state.error_flag = data[7] if len(data) > 7 else 0
    
    return True

def read_motor_status(bus, motor_controller: MotorController, motor_state: MotorState, max_attempts=3, timeout_ms=100):
    """
    Read motor status with retry mechanism and proper response filtering
    
    Args:
        bus: CAN bus object
        motor_controller: MotorController instance
        motor_state: MotorState instance to update
        max_attempts: Maximum number of read attempts
        timeout_ms: Timeout in milliseconds for each read attempt
    
    Returns:
        bool: True if valid motor status received, False otherwise
    """
    for attempt in range(max_attempts):
        try:
            # Read CAN frame
            msg = bus.recv(timeout=timeout_ms / 1000.0)
            if msg is None:
                print(f"Timeout waiting for motor response (attempt {attempt+1}/{max_attempts})")
                if attempt < max_attempts - 1:
                    time.sleep(0.02)
                continue

            can_id = msg.arbitration_id
            can_data = msg.data
            
            print(f"Received: ID={hex(can_id)}, Extended={msg.is_extended_id}, Data={[hex(b) for b in can_data]}")

            # Check if frame is extended (MIT Mode uses Standard CAN frames only)
            if msg.is_extended_id:
                print(f"Warning: Received Extended CAN frame (ID={hex(can_id)}). MIT Mode uses Standard frames only. Ignoring.")
                continue

            # Check if message is from our motor
            if can_id == motor_controller.controller_id:
                # Try to unpack the response
                if unpack_reply(can_data, motor_controller, motor_state):
                    print(f"Valid motor status received from Motor ID: {motor_state.motor_id}")
                    return True
                elif attempt < max_attempts - 1:
                    print("Invalid response format, retrying...")
                    time.sleep(0.02)  # Brief delay before retry
            else:
                print(f"Received message from different ID: {hex(can_id)} (expected: {hex(motor_controller.controller_id)})")

        except can.CanError as e:
            print(f"Error reading motor status: {e}")

    print("Failed to receive valid motor status after all attempts")
    return False

def request_motor_status(bus, controller_id):
    """
    Request motor status using stateless command
    According to the PDF: (0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC)
    But this is actually the enter mode command. For stateless read, it might be different.
    """
    # This might need to be adjusted based on actual motor firmware
    # Some motors respond automatically, others need specific status request
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    return send_can_message(bus, controller_id, data)

def print_menu():
    print("\n=== CubeMars Motor Control Menu ===")
    print("1. Enter MIT Mode")
    print("2. Exit MIT Mode")
    print("3. Send Control Command")
    print("4. Read Status")
    print("5. Zero Position")
    print("6. Run Auto Test")
    print("7. Request Status (Stateless)")
    print("q. Quit")
    print("Please enter your choice (1-7 or q):")

def get_float_input(prompt, min_val, max_val):
    """Get validated float input from user"""
    while True:
        try:
            value = float(input(f"Enter {prompt} ({min_val} to {max_val}): "))
            if min_val <= value <= max_val:
                return value
            print(f"Value must be between {min_val} and {max_val}")
        except ValueError:
            print("Please enter a valid number")

def select_motor_model():
    """Allow user to select motor model"""
    print("\nAvailable Motor Models:")
    models = list(MotorModel)
    for i, model in enumerate(models):
        specs = MOTOR_SPECS[model]
        print(f"{i+1}. {model.value} - Torque: {specs['torque_range'][0]} to {specs['torque_range'][1]} N·m")
    
    while True:
        try:
            choice = int(input(f"Select motor model (1-{len(models)}): "))
            if 1 <= choice <= len(models):
                return models[choice-1]
            print(f"Please enter a number between 1 and {len(models)}")
        except ValueError:
            print("Please enter a valid number")

def run_auto_test(bus, motor_controller: MotorController, motor_state: MotorState):
    """Run automatic test sequence with motor-specific parameters"""
    print(f"\n=== Running Auto Test for {motor_controller.model.value} ===")

    # Enter MIT mode
    print("Entering MIT Mode...")
    if not enter_mode(bus, motor_controller.controller_id):
        print("Failed to enter MIT mode")
        return
    time.sleep(0.2)

    # Zero position
    print("Zeroing position...")
    if not zero_position(bus, motor_controller.controller_id):
        print("Failed to zero position")
        return
    time.sleep(0.2)

    # Test positions scaled to motor capabilities
    max_test_pos = min(2.0, motor_controller.p_max * 0.5)  # Conservative test range
    test_positions = [0.0, max_test_pos, -max_test_pos, max_test_pos*0.5, -max_test_pos*0.5, 0.0]

    for pos in test_positions:
        print(f"\nTesting position: {pos:.2f} rad")
        
        # Set conservative control parameters
        motor_state.p_in = pos
        motor_state.v_in = 0.0
        motor_state.kp_in = 10.0  # Conservative gain
        motor_state.kd_in = 1.0   # Conservative damping
        motor_state.t_in = 0.0    # No feedforward torque

        # Send command
        if not pack_cmd(bus, motor_controller, motor_state):
            print(f"Failed to send command for position {pos}")
            continue
        
        time.sleep(1.0)  # Wait for movement

        # Read status
        if read_motor_status(bus, motor_controller, motor_state):
            print(f"  Actual Position: {motor_state.p_out:.3f} rad")
            print(f"  Velocity: {motor_state.v_out:.3f} rad/s") 
            print(f"  Torque: {motor_state.t_out:.3f} N·m")
            if hasattr(motor_state, 'temperature') and motor_state.temperature > 0:
                print(f"  Temperature: {motor_state.temperature}°C")
        else:
            print("  Failed to read motor status")

    print(f"\n=== Auto Test for {motor_controller.model.value} Completed ===")

def main():
    # Select motor model
    model = select_motor_model()
    
    # Get controller ID
    controller_id = int(input(f"Enter CAN controller ID (default 8): ") or "8")
    
    # Initialize motor controller
    motor_controller = MotorController(model, controller_id)
    motor_state = MotorState()

    # Get CAN interface parameters from user
    # For Linux, bustype='socketcan'. For Windows, common options are 'pcan', 'kvaser', 'vector'.
    # The channel depends on the hardware setup.
    bustype = input("Enter CAN bustype (e.g., 'socketcan', 'pcan', 'kvaser'): ")
    channel = input("Enter CAN channel (e.g., 'can0', 'pcanusb1'): ")
    bitrate = int(input("Enter bitrate (default 1000000): ") or "1000000")

    bus = None
    try:
        # Initialize CAN interface
        bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        print("CAN bus initialized successfully!")
        print(f"Motor: {motor_controller.model.value}")
        print(f"Controller ID: 0x{motor_controller.controller_id:X}")

        # Main control loop
        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            elif choice == '1':
                print("Entering MIT Mode...")
                enter_mode(bus, motor_controller.controller_id)
                time.sleep(0.1)

            elif choice == '2':
                print("Exiting MIT Mode...")
                exit_mode(bus, motor_controller.controller_id)
                time.sleep(0.1)

            elif choice == '3':
                print(f"\nControl Command for {motor_controller.model.value}:")
                motor_state.p_in = get_float_input("position", motor_controller.p_min, motor_controller.p_max)
                motor_state.v_in = get_float_input("velocity", motor_controller.v_min, motor_controller.v_max)
                motor_state.kp_in = get_float_input("kp", motor_controller.kp_min, motor_controller.kp_max)
                motor_state.kd_in = get_float_input("kd", motor_controller.kd_min, motor_controller.kd_max)
                motor_state.t_in = get_float_input("torque", motor_controller.t_min, motor_controller.t_max)

                print("Sending command...")
                pack_cmd(bus, motor_controller, motor_state)
                time.sleep(0.1)

            elif choice == '4':
                print("Reading motor status...")
                if read_motor_status(bus, motor_controller, motor_state):
                    print(f"\nMotor Status ({motor_controller.model.value}):")
                    print(f"  Motor ID: {motor_state.motor_id}")
                    print(f"  Position: {motor_state.p_out:.4f} rad")
                    print(f"  Velocity: {motor_state.v_out:.4f} rad/s")
                    print(f"  Torque: {motor_state.t_out:.4f} N·m")
                else:
                    print("Failed to read motor status")

            elif choice == '5':
                print("Zeroing position...")
                zero_position(bus, motor_controller.controller_id)
                time.sleep(0.1)

            elif choice == '6':
                run_auto_test(bus, motor_controller, motor_state)

            elif choice == '7':
                print("Requesting motor status...")
                request_motor_status(bus, motor_controller.controller_id)
                time.sleep(0.1)
                read_motor_status(bus, motor_controller, motor_state)

            else:
                print("Invalid choice! Please select 1-7 or q")

            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        if bus:
            try:
                print("\nCleaning up...")
                exit_mode(bus, motor_controller.controller_id)
                bus.shutdown()
                print("CAN interface closed successfully")
            except Exception as e:
                print(f"Error during cleanup: {e}")

if __name__ == "__main__":
    main()
