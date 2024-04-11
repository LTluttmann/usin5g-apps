#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : jrobot_can.py
Description : 极创底盘CAN通讯
Author : zhangxu
Create Time : 2022-10-31 
-------------------------------------------------------------------------------
Change Activity:
        zhangxu:更改内容
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'


import os
import can
import rospy
from custom_msgs.msg import Float32ListStamped, Int32ListStamped
import struct


CHANNEL = rospy.get_param('motor_can/channel', 'can0')
BITRATE = rospy.get_param('motor_can/bitrate', 250000)
CAN_ID = rospy.get_param('motor_can/id', 0x601)
RATE_ENCODER = rospy.get_param('motor_can/rate_encoder', 20)
MAX_RPM = rospy.get_param('motor_can/max_rpm', 3000)


def init_can():
    """初始化CAN."""
    os.system("sudo modprobe can; sudo modprobe can_raw; sudo modprobe mttcan")
    bash_can_down = f"sudo ip link set down {CHANNEL}"
    bash_can_set = f"sudo ip link set {CHANNEL} type can bitrate {BITRATE}"
    bash_can_up = f"sudo ip link set up {CHANNEL}"
    os.system(bash_can_down)
    os.system(bash_can_set)
    os.system(bash_can_up)
    # sudo ip link set can0 type can bitrate 250000 sample-point 0.8 dbitrate 2000000 dsample-point 0.75 fd on restart-ms 100


class CanNode:
    def __init__(self):
        rospy.init_node("jrobot_can")
        rospy.loginfo("Starting CanNode as jrobot_can.")

        self.bus = can.Bus(
            interface='socketcan',
            channel=CHANNEL,
            bitrate=BITRATE,
            receive_own_messages=False)
        self.bus.flush_tx_buffer()

        self.encoder = Int32ListStamped()
        self.encoder.header = rospy.Header(stamp=rospy.Time.now())
        self.encoder.values = [0, 0]

        self.rpm = Int32ListStamped()
        self.rpm.header = rospy.Header(stamp=rospy.Time.now())
        self.rpm.values = [0, 0]

        rospy.Subscriber("cmd_motor_rpm", Float32ListStamped,
                         self.callback_cmd_motor_rpm)
        motor_encoder_pub = rospy.Publisher(
            "motor_encoder", Int32ListStamped, queue_size=10)
        motor_rpm_pub = rospy.Publisher(
            "motor_rpm", Int32ListStamped, queue_size=10)
        rate = rospy.Rate(RATE_ENCODER)
        while not rospy.is_shutdown():
            self.encoder.header.stamp = rospy.Time.now()
            self.rpm.header.stamp = rospy.Time.now()
            try:
                self.encoder.values[0] = self.get_encoder(1)
                self.encoder.values[1] = self.get_encoder(2)
                motor_encoder_pub.publish(self.encoder)
                self.rpm.values[0] = self.get_rpm(1)
                self.rpm.values[1] = self.get_rpm(2)
                motor_rpm_pub.publish(self.rpm)
            except can.CanError as e:
                print(e)
            rate.sleep()

    def callback_cmd_motor_rpm(self, cmd_rpm):
        """
        转速命令回调.

        :param cmd_rpm: [Float32ListStamped]转速命令
        :returns: [类型]描述
        :raises: no exception
        """
        message_l = can.Message(arbitration_id=CAN_ID, extended_id=False,
                                data=self.write_rpm(1, cmd_rpm.values[0]))
        message_r = can.Message(arbitration_id=CAN_ID, extended_id=False,
                                data=self.write_rpm(2, cmd_rpm.values[1]))

        try:
            self.bus.send(message_l, timeout=0.2)
            self.bus.send(message_r, timeout=0.2)
        except can.CanError as e:
            print(e)

    def get_encoder(self, channel):
        """
        获取编码器数据.

        :param channel: [int]电机通道
        :returns: [int]数据
        :raises: no exception
        """
        message = can.Message(arbitration_id=CAN_ID, extended_id=False,
                              data=self.read_encoder(channel))

        self.bus.flush_tx_buffer()
        self.bus.send(message, timeout=0.2)

        for msg in self.bus:
            #print(f"{msg.arbitration_id:X}: {msg.data.hex()}")
            if msg.arbitration_id != 0x701:
                _data = struct.unpack("<BHBi", msg.data)
                if _data[1] == 0x2104 and _data[2] == int(channel):
                    value = _data[3]
                    #print(channel, value)
                    return value
                    break
            else:
                pass

    def get_rpm(self, channel):
        """
        获取电机速度.

        :param channel: [int]电机通道
        :returns: [int]数据
        :raises: no exception
        """
        message = can.Message(arbitration_id=CAN_ID, extended_id=False,
                              data=self.read_rpm(channel))

        self.bus.send(message, timeout=0.2)
        for msg in self.bus:
            #print(f"{msg.arbitration_id:X}: {msg.data.hex()}")
            if msg.arbitration_id != 0x701:
                _data = struct.unpack("<BHBhH", msg.data)
                if _data[1] == 0x210a and _data[2] == int(channel):
                    value = _data[3]
                    #print(channel, value)
                    return value
                    break
            else:
                pass

    def write_rpm(self, channel, rpm):
        """
        设定电机速度.

        :param channel: [int]电机通道
        :param rpm: [int]转速
        :returns: [list]报文数据
        :raises: no exception
        """
        cmd_rpm = rpm*1000/MAX_RPM
        _data = b'\x23\x00\x20'+struct.pack("B", channel) + \
            struct.pack("<h", int(cmd_rpm))+b'\x00\x00'
        # print(_data.hex())

        return _data

    def read_encoder(self, channel):
        """
        读取编码器.

        :param channel: [int]电机通道
        :returns: [list]报文数据
        :raises: no exception
        """
        # print(rpm)
        _data = b'\x40\x04\x21'+struct.pack("B", channel) + b'\x00\x00\x00\x00'
        # print(_data.hex())
        return _data

    def read_rpm(self, channel):
        """
        读取电机速度.

        :param channel: [int]电机通道
        :returns: [list]报文数据
        :raises: no exception
        """
        # print(rpm)
        _data = b'\x40\x0a\x21'+struct.pack("B", channel) + b'\x00\x00\x00\x00'
        # print(_data.hex())
        return _data

    def write_message(self, name, data, channel=None):
        """
        写入数据.

        :param name: [str]数据名
        :param data: [类型]数据
        :param channel: [int]通道
        :returns: [类型]描述
        :raises: no exception
        """
        pass

    def read_message(self, name, data, channel=None):
        """
        读取数据.

        :param name: [str]数据名
        :param data: [类型]数据
        :param channel: [int]通道
        :returns: [类型]读取数据
        :raises: no exception
        """


if __name__ == "__main__":
    init_can()
    jrobot_can = CanNode()
    rospy.spin()
