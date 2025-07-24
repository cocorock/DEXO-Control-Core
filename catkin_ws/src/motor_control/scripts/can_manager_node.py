#!/usr/bin/env python

import rospy
import can
import struct
from motor_control.msg import MotorCommand, MotorState

# Constants for AK80-8 motor
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -8.0, 8.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -144.0, 144.0

# Motor IDs mapping from name to CAN ID
MOTOR_IDS = {
    'hip_right': 0x01,
    'knee_right': 0x02,
    'hip_left': 0x03,
    'knee_left': 0x04
}
# Reverse mapping from CAN ID to name for receiving messages
MOTOR_NAMES = {v: k for k, v in MOTOR_IDS.items()}

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return int((x - offset) * 4095.0 / span)
    elif bits == 16:
        return int((x - offset) * 65535.0 / span)
    return 0

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return (float(x_int) * span / 4095.0) + offset
    elif bits == 16:
        return (float(x_int) * span / 65535.0) + offset
    return 0.0

class CanManagerNode:
    def __init__(self):
        rospy.init_node('can_manager_node')
        rospy.loginfo("CAN Manager Node started.")

        can_channel = rospy.get_param('~can_channel', 'can0')
        can_bustype = rospy.get_param('~can_bustype', 'socketcan')

        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype=can_bustype, bitrate=1000000)
            rospy.loginfo("Successfully initialized CAN bus on channel '{}' with bustype '{}'".format(can_channel, can_bustype))
        except Exception as e:
            rospy.logerr("Failed to initialize CAN bus: {}".format(e))
            rospy.signal_shutdown("Failed to initialize CAN bus")
            return

        # Create publishers and subscribers for each motor
        self.publishers = {}
        for name, motor_id in MOTOR_IDS.items():
            # Publishes motor state FROM the bus TO the logic node
            self.publishers[motor_id] = rospy.Publisher('/{}/motor_state'.format(name), MotorState, queue_size=10)
            # Subscribes to motor commands FROM the logic node TO the bus
            rospy.Subscriber('/{}/motor_command'.format(name), MotorCommand, self.command_callback, callback_args=motor_id)

        # Start a timer to read from the CAN bus
        rospy.Timer(rospy.Duration(0.001), self.read_can_messages) # Read at 1kHz

    def command_callback(self, msg, motor_id):
        """Packs and sends a CAN message when a MotorCommand is received."""
        p_des = max(min(msg.p_in, P_MAX), P_MIN)
        v_des = max(min(msg.v_in, V_MAX), V_MIN)
        kp = max(min(msg.kp_in, KP_MAX), KP_MIN)
        kd = max(min(msg.kd_in, KD_MAX), KD_MIN)
        t_ff = max(min(msg.t_in, T_MAX), T_MIN)

        p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
        v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
        kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

        buf = bytearray(8)
        buf[0] = p_int >> 8
        buf[1] = p_int & 0xFF
        buf[2] = v_int >> 4
        buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
        buf[4] = kp_int & 0xFF
        buf[5] = kd_int >> 4
        buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
        buf[7] = t_int & 0xFF

        can_msg = can.Message(arbitration_id=motor_id, data=buf, is_extended_id=False)
        try:
            self.bus.send(can_msg)
        except can.CanError:
            rospy.logwarn("Failed to send CAN message for motor ID {}".format(motor_id))

    def read_can_messages(self, event=None):
        """Periodically reads from the CAN bus and publishes MotorState messages."""
        try:
            msg = self.bus.recv(timeout=0.0) # Non-blocking read
            if msg is not None and msg.arbitration_id in self.publishers:
                motor_id = msg.data[0]
                p_int = (msg.data[1] << 8) | msg.data[2]
                v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
                i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]

                state_msg = MotorState()
                state_msg.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
                state_msg.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
                state_msg.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12)

                self.publishers[msg.arbitration_id].publish(state_msg)
        except can.CanError:
            rospy.logwarn("Error receiving CAN message")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CanManagerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
