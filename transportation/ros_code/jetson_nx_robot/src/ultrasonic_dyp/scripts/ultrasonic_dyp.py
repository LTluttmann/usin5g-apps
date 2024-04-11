#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : ultrasonic_dyp.py
Description : 超声波传感器
Author : zhangxu
Create Time : 2022-11-02 
-------------------------------------------------------------------------------
Change Activity:
        zhangxu:更改内容
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'

import rospy
import time
from custom_msgs.msg import Float32ListStamped
from smbus2 import SMBus


bus_sensors = rospy.get_param('ultrasonic/i2c_bus', {8: [0x68, ], })

addrs_sensor = [0xa1, 0xa2, 0xa3, 0xa4]
cmd_write = [0xd1, 0xd2, 0xd3, 0xd4]


class I2cBus(object):
    """
    I2C BUS.

    :param bus_numb: [int]i2c bus 编号
    :param addrs: [list]传感器地址列表
    """

    def __init__(self, bus_numb, addrs_list):
        if isinstance(bus_numb, int):
            self.numb = bus_numb
        elif isinstance(bus_numb, str):
            self.numb = int(bus_numb)
        try:
            self.bus = SMBus(self.numb)
            rospy.loginfo(f"open i2c bus {bus_numb} ok")
        except Exception as e:
            rospy.loginfo(e)
            self.bus.close()
        self.addrs = addrs_list
        self.values = []

    def start_sensors(self):
        """启动传感器."""
        for _addr in self.addrs:
            self.bus.write_byte_data(_addr, addrs_sensor[0], cmd_write[3])

    def get_values(self):
        """
        获取传感器数据.

        :returns: [list]传感器数据,单位m
        :raises: no exception
        """
        self.values = []
        for _addr in self.addrs:
            for i in range(4):
                _block = self.bus.read_i2c_block_data(
                    _addr, addrs_sensor[i], 2)
                _value = ((_block[0] << 8)+_block[1])/1000
                if _value > 10:
                    _value = 0.0
                self.values.append(_value)
        return self.values


class UltrasonicNode:
    def __init__(self):
        rospy.init_node("ultrasonic_dyp")
        rospy.loginfo("Starting UltrasonicNode as untrasonic_dyp.")
        self.sensor_pub = rospy.Publisher(
            "ultrasonic_data", Float32ListStamped, queue_size=10)
        rate = rospy.Rate(5)
        self.data = Float32ListStamped()
        self.data.header = rospy.Header()
        self.data.values = []

        self.bus = []
        for key, val in bus_sensors.items():
            _bus = I2cBus(key, val)
            self.bus.append(_bus)

        while not rospy.is_shutdown():
            try:
                self.start()
                time.sleep(0.2)
                self.get_data()
                self.data.header.stamp = rospy.Time.now()
                self.sensor_pub.publish(self.data)
                rate.sleep()
            except Exception as e:
                rospy.loginfo(e)

    def start(self):
        """启动传感器."""
        for _bus in self.bus:
            _bus.start_sensors()

    def get_data(self):
        """获取传感器数据."""
        self.data.values = []

        for _bus in self.bus:
            self.data.values.extend(_bus.get_values())
        # rospy.loginfo(self.data.values)


if __name__ == "__main__":
    ultrasonic_dyp = UltrasonicNode()
    for _bus in ultrasonic_dyp.bus:
        _bus.bus.close()
