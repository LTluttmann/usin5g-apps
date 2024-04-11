#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : joy_ctrl.py
Description : 手柄控制
Author : zhangxu
Create Time : 2022-10-28 
-------------------------------------------------------------------------------
Change Activity:
        zhangxu:更改内容
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'


import time
import rospy
from std_msgs.msg import ColorRGBA, Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import Joy

axes_lid = rospy.get_param("joy/axes_lid", 1)
#button_lid_open = rospy.get_param("joy/button_lid_open", 3)
#button_lid_close = rospy.get_param("joy/button_lid_close", 0)
button_light_r = rospy.get_param("joy/button_light_r", 1)
button_light_g = rospy.get_param("joy/button_light_g", 0)
button_light_b = rospy.get_param("joy/button_light_b", 2)
axes_light_turn = rospy.get_param("joy/axes_light_turn", 6)


def lid_ctrl(cmd):
    """
    开关盖控制.

    :param cmd: [bool]是否开关盖
    :returns: [bool]是否成功
    :raises: no exception
    """
    rospy.wait_for_service("lid_ctrl")
    try:
        srv_lid = rospy.ServiceProxy("lid_ctrl", SetBool)
        resp = srv_lid(cmd)
        rospy.loginfo(resp.message)
        return resp.success
    except rospy.ServiceException as e:
        print(e)


class JoyNode:
    def __init__(self):
        rospy.init_node("joy_ctrl")
        rospy.loginfo("Starting JoyNode as joy_ctrl.")

        rospy.Subscriber("joy", Joy, self.callback_joy)
        self.light_pub = rospy.Publisher("light", ColorRGBA, queue_size=10)
        self.light_turn_l_pub = rospy.Publisher(
            "light/turn_l", Bool, queue_size=10)
        self.light_turn_r_pub = rospy.Publisher(
            "light/turn_r", Bool, queue_size=10)

        self.light_color = ColorRGBA()
        self.light_turn_l = Bool()
        self.light_turn_r = Bool()

        self.joy_lid_last = 0

    def callback_joy(self, data):
        """
        手柄回调.

        :param data: [Joy]手柄值
        :returns: [类型]描述
        :raises: no exception
        """
        self.light_color.r = data.buttons[button_light_r]
        self.light_color.g = data.buttons[button_light_g]
        self.light_color.b = data.buttons[button_light_b]
        self.light_pub.publish(self.light_color)

        _turn = data.axes[axes_light_turn]
        if _turn == 1.0:
            self.light_turn_l.data = True
        elif _turn == -1.0:
            self.light_turn_r.data = True
        else:
            self.light_turn_l.data = False
            self.light_turn_r.data = False
        self.light_turn_l_pub.publish(self.light_turn_l)
        self.light_turn_r_pub.publish(self.light_turn_r)

        joy_lid = data.axes[axes_lid]
        if joy_lid == 1.0 and self.joy_lid_last == 0.0:
            lid_ctrl(True)
        elif joy_lid == -1.0 and self.joy_lid_last == 0.0:
            lid_ctrl(False)
        else:
            pass
        self.joy_lid_last = joy_lid


if __name__ == "__main__":
    joy_ctrl = JoyNode()
    rospy.spin()
