#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : light_ctrl.py
Description : 灯光控制
Author : zhangxu
Create Time : 2022-10-27 
-------------------------------------------------------------------------------
Change Activity:
        zhangxu:更改内容
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'


import time
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import ColorRGBA, Bool


light_pin_r = rospy.get_param("light/pin_r", 33)
light_pin_g = rospy.get_param("light/pin_g", 32)
light_pin_b = rospy.get_param("light/pin_b", 15)
light_pin_turn_l = rospy.get_param("light/pin_turn_l", 16)
light_pin_turn_r = rospy.get_param("light/pin_turn_r", 18)


def init_io():
    """初始化."""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(light_pin_r, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(light_pin_g, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(light_pin_b, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(light_pin_turn_l, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(light_pin_turn_r, GPIO.OUT, initial=GPIO.LOW)


def set_light_on(pin, color_value):
    """
    设置灯光亮灭.

    :param pin: [int]灯管脚
    :param color_value: [float32]颜色值
    :returns: [类型]描述
    :raises: no exception
    """
    if color_value == 0:
        GPIO.output(pin, GPIO.LOW)
    else:
        GPIO.output(pin, GPIO.HIGH)


def callback_light(color):
    """
    灯光回调.

    :param color: [ColorRGBA]灯光颜色
    :returns: [类型]描述
    :raises: no exception
    """
    rospy.loginfo(color)
    set_light_on(light_pin_r, color.r)
    set_light_on(light_pin_g, color.g)
    set_light_on(light_pin_b, color.b)


def callback_turn_l(data):
    """
    左转向灯回调.

    :param data: [Bool]开关
    :returns: [类型]描述
    :raises: no exception
    """
    set_light_on(light_pin_turn_l, data.data)


def callback_turn_r(data):
    """
    右转向灯回调.

    :param data: [Bool]开关
    :returns: [类型]描述
    :raises: no exception
    """
    set_light_on(light_pin_turn_r, data.data)


class LightNode:
    def __init__(self):
        rospy.init_node("light")
        rospy.loginfo("Starting LightNode as light.")
        rospy.Subscriber("light", ColorRGBA, callback_light)
        rospy.Subscriber("light/turn_l", Bool, callback_turn_l)
        rospy.Subscriber("light/turn_r", Bool, callback_turn_r)


if __name__ == "__main__":
    init_io()
    light = LightNode()
    rospy.spin()
