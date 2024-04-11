#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : ultrasonic.py
Description : 超声波传感器
Author : zhangxu
Create Time : 2022-11-03 
-------------------------------------------------------------------------------
Change Activity:
        zhangxu:更改内容
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'

import rospy
from sensor_msgs.msg import Range
from custom_msgs.msg import Float32ListStamped
import math

index = rospy.get_param("ultrasonic/data_index",
                        ['f0', 'fl', 'fr', 'l', 'r', 'r0', 'rl', 'rr'])
len_index = len(index)
fov = rospy.get_param("ultrasonic/field_of_view", 60.0)
min_range = rospy.get_param("ultrasonic/min_range", 0.28)
max_range = rospy.get_param("ultrasonic/max_range", 4.5)


class UltrasonicNode:
    def __init__(self):
        rospy.init_node("ultrasonic")
        rospy.loginfo("Starting UltrasonicNode as ultrasonic.")
        rospy.Subscriber("ultrasonic_data", Float32ListStamped,
                         self.callback_ultrasonic_data)
        self.ranges = []
        self.pubs = []
        self.init_pubs()

    def callback_ultrasonic_data(self, data):
        """
        传感器数据回调.

        :param data: [Float32ListStamped]传感器数据
        :returns: [类型]描述
        :raises: no exception
        """
        _len = len(data.values)
        for i in range(len_index):
            if i < _len:
                self.ranges[i].header.stamp = data.header.stamp
                self.ranges[i].range = data.values[i]
                self.pubs[i].publish(self.ranges[i])
            else:
                print("索引超出数据长度")
                break

    def init_pubs(self):
        """初始化传感器发布."""
        for i in range(len_index):
            _msg = Range()
            _msg.header = rospy.Header()
            _msg.header.frame_id = "ultrasonic_"+index[i]
            _msg.radiation_type = Range.ULTRASOUND
            _msg.field_of_view = fov * math.pi / 180
            _msg.min_range = min_range
            _msg.max_range = max_range
            self.ranges.append(_msg)
            _pub = rospy.Publisher(
                "ultrasonic/"+index[i], Range, queue_size=10)
            self.pubs.append(_pub)


if __name__ == "__main__":
    ultrasonic = UltrasonicNode()
    rospy.spin()
