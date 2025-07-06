#!/usr/bin/env python

import rospy
from motor_control.msg import MotorCommand, MotorState
import struct
import time
import can

# Constants for AK80-8 motor
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -8.0
V_MAX = 8.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -144.0
T_MAX = 144.0

# Motor IDs
MOTOR_IDS = {
    'hip_right': 0x01,
    'knee_right': 0x02,
    'hip_left': 0x03,
    'knee_left': 0x04
}

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

class MotorController:
    def __init__(self, motor_id, can_channel, can_bustype):
        self.motor_id = motor_id
        self.p_in = 0.0
        self.v_in = 0.0
        self.kp_in = 0.0
        self.kd_in = 0.50
        self.t_in = 0.0

        # Measured values
        self.p_out = 0.0
        self.v_out = 0.0
        self.t_out = 0.0

        self.state_pub = rospy.Publisher('/motor_state_' + str(self.motor_id), MotorState, queue_size=10)
        self.command_sub = rospy.Subscriber('/motor_command_' + str(self.motor_id), MotorCommand, self.command_callback)

        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype=can_bustype, bitrate=1000000)
        except Exception as e:
            rospy.logerr(f"Failed to initialize CAN bus: {e}")
            rospy.signal_shutdown("Failed to initialize CAN bus")

    def command_callback(self, msg):
        self.p_in = msg.p_in
        self.v_in = msg.v_in
        self.kp_in = msg.kp_in
        self.kd_in = msg.kd_in
        self.t_in = msg.t_in

        rospy.loginfo(f"Received command for motor {self.motor_id}: p={self.p_in}, v={self.v_in}, kp={self.kp_in}, kd={self.kd_in}, t={self.t_in}")
        self.send_command()

    def send_command(self):
        p_des = max(min(self.p_in, P_MAX), P_MIN)
        v_des = max(min(self.v_in, V_MAX), V_MIN)
        kp = max(min(self.kp_in, KP_MAX), KP_MIN)
        kd = max(min(self.kd_in, KD_MAX), KD_MIN)
        t_ff = max(min(self.t_in, T_MAX), T_MIN)

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

        msg = can.Message(arbitration_id=self.motor_id, data=buf, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            rospy.logerr("Failed to send CAN message")

    def read_motor_state(self):
        try:
            msg = self.bus.recv(timeout=0.1)
            if msg is not None and msg.arbitration_id == self.motor_id:
                id_received = msg.data[0]
                p_int = (msg.data[1] << 8) | msg.data[2]
                v_int = (msg.data[3] << 4) | (msg.data[4] >> 4)
                i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5]

                self.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
                self.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
                self.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12)

                state_msg = MotorState()
                state_msg.p_out = self.p_out
                state_msg.v_out = self.v_out
                state_msg.t_out = self.t_out
                self.state_pub.publish(state_msg)
        except can.CanError:
            rospy.logerr("Failed to receive CAN message")

def main():
    rospy.init_node('motor_control_node')
    rospy.loginfo("Motor control node started")

    motor_to_control = rospy.get_param('~motor', 'hip_right')
    can_channel = rospy.get_param('~can_channel', 'can0')
    can_bustype = rospy.get_param('~can_bustype', 'socketcan')

    if motor_to_control not in MOTOR_IDS:
        rospy.logerr(f"Invalid motor name: {motor_to_control}")
        return

    motor_id = MOTOR_IDS[motor_to_control]
    motor_controller = MotorController(motor_id, can_channel, can_bustype)

    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        motor_controller.read_motor_state()
        rate.sleep()

if __name__ == '__main__':
    main()
