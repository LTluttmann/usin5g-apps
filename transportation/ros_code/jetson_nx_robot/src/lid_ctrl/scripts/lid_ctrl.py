#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : lid_ctrl.py
Description : 开关盖控制
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
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool


lid_pin_open = rospy.get_param("lid/pin_open", 11)
lid_pin_close = rospy.get_param("lid/pin_close", 12)
lid_pin_unclosed = rospy.get_param("lid/pin_unclosed", 13)
time_open = rospy.get_param("lid/time_open", 3)
time_close = rospy.get_param("lid/time_close", 3)


def init_io():
    """初始化."""
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(lid_pin_open, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(lid_pin_close, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(lid_pin_unclosed, GPIO.IN)


def open_lid():
    """开盖."""
    try:
        GPIO.output(lid_pin_open, GPIO.HIGH)
        GPIO.output(lid_pin_close, GPIO.LOW)
        time.sleep(time_open)
        GPIO.output(lid_pin_open, GPIO.LOW)
        GPIO.output(lid_pin_close, GPIO.LOW)
    except Exception as e:
        rospy.loginfo(e)


def close_lid():
    """关盖."""
    try:
        GPIO.output(lid_pin_open, GPIO.LOW)
        GPIO.output(lid_pin_close, GPIO.HIGH)
        time.sleep(time_close)
        GPIO.output(lid_pin_open, GPIO.LOW)
        GPIO.output(lid_pin_close, GPIO.LOW)
    except Exception as e:
        rospy.loginfo(e)


def scan_lid_unclosed():
    """检测盖子是否打开."""
    if GPIO.input(lid_pin_unclosed) == GPIO.HIGH:
        return False
    elif GPIO.input(lid_pin_unclosed) == GPIO.LOW:
        return True
    else:
        rospy.loginfo("检测盖子状态失败")
        return False


def callback_lid_ctrl(ctrl_open):
    """
    服务回调.

    :param open: [SetBool]开关盖
    :returns: [SetBoolResponse]是否成功
    :raises: no exception
    """
    _success = False
    _message = ""
    if ctrl_open.data:
        open_lid()
        _success = True
        _message = "lid open OK"
    else:
        close_lid()
        _success = True
        _message = "lid close OK"
    return SetBoolResponse(_success, _message)


class LidNode:
    def __init__(self):
        rospy.init_node("lid_ctrl")
        rospy.loginfo("Starting LidNode as lid_ctrl.")
        self.lid_unclosed = Bool()
        rate = rospy.Rate(5)
        self.lid_unclosed_pub = rospy.Publisher(
            "lid/unclosed", Bool, queue_size=10)
        handler_lid_ctrl = rospy.Service(
            "lid_ctrl", SetBool, callback_lid_ctrl)

        while not rospy.is_shutdown():
            self.lid_unclosed.data = scan_lid_unclosed()
            self.lid_unclosed_pub.publish(self.lid_unclosed)
            rate.sleep()


if __name__ == "__main__":
    init_io()
    lid_ctrl = LidNode()
    rospy.spin()
    #GPIO.cleanup([lid_pin_open, lid_pin_close])
