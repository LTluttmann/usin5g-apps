#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : diff_drive.py
Description : 差速驱动控制
Author : zhangxu
Create Time : 2022-10-28 
-------------------------------------------------------------------------------
Change Activity:
        2022.10.28:
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'

from math import pi, sin, cos
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Pose2D, Pose, TransformStamped
from nav_msgs.msg import Odometry
from custom_msgs.msg import Float32ListStamped, Int32ListStamped


#classes#######################################################################

class MotorEncoder(object):
    """
    电机编码器.

    :param resolution: [int]编码器分辨率
    :param count_max: [int]编码器最大计数
    """

    def __init__(self, resolution, count_max, time_out=3.0):
        self._resolution = resolution
        self._count_max = count_max
        self._count_max_half = int(count_max / 2)
        self._time_last = 0.0
        self._time_current = 0.0
        self._count_last = 0
        self._count_current = 0
        self._time_out = time_out

    def get_count_delta(self, time, count):
        """
        获取编码器计数变化.

        :param time: [float]时间
        :param count: [int]编码器计数
        :returns: ([int]计数变化,[float]时间间隔)
        :raises: no exception
        """
        self._time_current = time
        self._count_current = count
        _count_diff = self._count_current-self._count_last
        _time_delta = self._time_current-self._time_last
        if self._time_last == 0.0 or _time_delta == 0.0 or _time_delta > self._time_out:
            self._time_last = time
            self._count_last = self._count_current
            return 0, 0.0
        elif _count_diff > self._count_max_half:
            _count_delta = _count_diff-self._count_max
        elif _count_diff < 0 and -_count_diff > self._count_max_half:
            _count_delta = _count_diff+self._count_max
        else:
            _count_delta = _count_diff
        self._time_last = self._time_current
        self._count_last = self._count_current

        return _count_delta, _time_delta


class DiffDrive(object):
    """
    差速驱动.

    :param wheel_radius: [float]轮子半径
    :param wheel_base: [float]两轮中心距
    :param resolution: [int]编码器分辨率
    :param count_max: [int]编码器最大计数
    :param gear_ratio: [float]减速比
    :param reverse_l: [bool]左轮反向
    :param reverse_r: [bool]右轮反向
    """

    def __init__(self, wheel_radius, wheel_base, resolution, count_max, gear_ratio, reverse_l, reverse_r, time_out):
        self._wheel_radius = wheel_radius
        self._wheel_base = wheel_base
        self._gear_ratio = gear_ratio
        self._reverse_l = reverse_l
        self._reverse_r = reverse_r
        self.encoder_l = MotorEncoder(resolution, count_max, time_out)
        self.encoder_r = MotorEncoder(resolution, count_max, time_out)
        self._distance_per_plus = 2*pi*wheel_radius/resolution/gear_ratio
        self.now = None
        self.dl_l = 0.0  # 弧长增量L
        self.dl_r = 0.0  # 弧长增量R
        self.v_l = 0.0  # 轮速L
        self.v_r = 0.0  # 轮速R
        self.filter_v = MeanFilter(5)
        self.filter_w = MeanFilter(5)
        self.rpm_req_l = 0.0  # 请求轮速L
        self.rpm_req_r = 0.0  # 请求轮速R

        self.p = Pose2D()  # 2D位姿
        self.d_p_b = Pose2D()  # 2D位姿增量-base_link坐标系
        self.d_p_o = Pose2D()  # 2D位姿增量-odom坐标系
        self.pose = Pose()  # 3D位姿
        self.twist = Twist()  # 线速度角速度

        self.odom = Odometry()
        self.tf = TransformStamped()
        self.cmd_vel = Twist()

        self.init_odom()
        self.init_tf()

    @property
    def rpm_motor_l(self):
        """请求电机转速L."""
        return self.rpm_req_l*self._gear_ratio

    @property
    def rpm_motor_r(self):
        """请求电机转速R."""
        return self.rpm_req_r*self._gear_ratio

    def calculat_encoder(self, time, count_l, count_r, filter=True):
        """
        通过编码器计数计算.

        :param time: [float]编码器当前时间
        :param count_l: [int]左轮计数
        :param count_r: [int]右轮计数
        :param filter: [bool]是否对计算出的速度进行滤波
        :returns: [类型]描述
        :raises: no exception
        """
        self.now = rospy.Time.from_seconds(time)
        dc_l, dt = self.encoder_l.get_count_delta(time, count_l)
        dc_r, dt = self.encoder_r.get_count_delta(time, count_r)
        #print(dc_l, dc_r, count_l, count_r)
        # print(self.encoder_l.__dict__)

        if dt == 0:
            return None
        if self._reverse_l:
            dc_l = -dc_l
        if self._reverse_r:
            dc_r = -dc_r
        self.dl_l = dc_l*self._distance_per_plus
        self.dl_r = dc_r*self._distance_per_plus
        self.v_l = self.dl_l/dt
        self.v_r = self.dl_r/dt

        dth = (self.dl_r-self.dl_l)/self._wheel_base
        # 直线行驶
        if dth == 0.0:
            w = 0.0
            v = self.v_l
            self.d_p_b.x = self.dl_l
            self.d_p_b.y = 0.0
            self.d_p_b.theta = 0.0
        else:
            r = (self.dl_l+self.dl_r)*self._wheel_base/2/(self.dl_r-self.dl_l)
            w = dth/dt
            v = w*r
            self.d_p_b.x = r*sin(dth)
            self.d_p_b.y = r*(1-cos(dth))
            self.d_p_b.theta = dth

        self.d_p_o.x = self.d_p_b.x * \
            cos(self.p.theta)-self.d_p_b.y*sin(self.p.theta)
        self.d_p_o.y = self.d_p_b.x * \
            sin(self.p.theta)+self.d_p_b.y*cos(self.p.theta)
        self.d_p_o.theta = self.d_p_b.theta

        self.p.x += self.d_p_o.x
        self.p.y += self.d_p_o.y
        self.p.theta += self.d_p_o.theta

        self.p.theta = self.p.theta % (2*pi)

        self.pose.position.x = self.p.x
        self.pose.position.y = self.p.y
        self.pose.position.z = 0
        self.pose.orientation.z = sin(self.p.theta / 2.0)
        self.pose.orientation.w = cos(self.p.theta / 2.0)

        #self.pose.orientation=tf.transformations.quaternion_from_euler(0, 0, self.p.theta)

        if filter:
            self.twist.linear.x = self.filter_v.filter(v)
            self.twist.angular.z = self.filter_w.filter(w)
        else:
            self.twist.linear.x = v
            self.twist.angular.z = w
        return True

    def twist_to_wheel_speed(self, twist):
        """
        角速度线速度转左右轮速rpm.

        :param twist: [twist]角速度线速度请求
        :returns: ([float]左轮速度rpm,[float]右轮速度rpm)
        :raises: no exception
        """
        a = twist.linear.x/self._wheel_radius
        b = twist.angular.z*self._wheel_base/2/self._wheel_radius
        c = 30/pi
        self.rpm_req_l = (a-b)*c
        self.rpm_req_r = (a+b)*c
        if self._reverse_l:
            self.rpm_req_l = -self.rpm_req_l
        if self._reverse_r:
            self.rpm_req_r = -self.rpm_req_r
        return self.rpm_req_l, self.rpm_req_r

    def init_odom(self, frame_id="odom", child_frame_id="base_link"):
        """
        初始化odom.

        :param frame_id: [str]odom frame_id
        :param child_frame_id: [str]odom child_frame_id
        :returns: [类型]描述
        :raises: no exception
        """
        self.odom.header.frame_id = frame_id
        self.odom.child_frame_id = child_frame_id

    def init_tf(self, frame_id="odom", child_frame_id="base_link"):
        """
        初始化tf.

        :param frame_id: [str]tf frame_id
        :param child_frame_id: [str]tf child_frame_id
        :returns: [类型]描述
        :raises: no exception
        """
        self.tf.header.frame_id = frame_id
        self.tf.child_frame_id = child_frame_id

    def update_odom_tf(self):
        """更新odom,tf."""

        self.odom.header.stamp = self.now
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.tf.header.stamp = self.now
        self.tf.transform.translation = self.pose.position
        self.tf.transform.rotation = self.pose.orientation


class MeanFilter(object):
    """
    均值滤波.

    :param num: [int]数量
    """

    def __init__(self, num):
        self.store = []
        self._num = num

    def filter(self, value):
        """
        均值滤波.

        :param value: [float]原始数据
        :returns: [float]滤波后数据
        :raises: no exception
        """
        self.store.append(value)
        if len(self.store) == self._num+1:
            self.store.pop(0)
            return sum(self.store)/self._num
        else:
            return sum(self.store)/len(self.store)


class DiffDriveNode:
    def __init__(self):
        rospy.init_node("diff_drive")
        rospy.loginfo("Starting DiffDriveNode as diff_drive.")

        self.drive = DiffDrive(
            rospy.get_param("diff_drive/wheel_radius", 0.155),
            rospy.get_param("diff_drive/wheel_base", 0.506),
            rospy.get_param("diff_drive/resolution", 4096),
            2**rospy.get_param("diff_drive/count_bits", 32),
            rospy.get_param("diff_drive/gear_ratio", 47.0),
            rospy.get_param("diff_drive/reverse_l", False),
            rospy.get_param("diff_drive/reverse_r", True),
            rospy.get_param("diff_drive/time_out", 3.0),
        )

        rospy.Subscriber("motor_encoder", Int32ListStamped,
                         self.callback_encoder)
        rospy.Subscriber("cmd_vel", Twist, self.callback_vel)
        self.odom_pub = rospy.Publisher("odom_drive", Odometry, queue_size=10)
        self.motor_rpm_pub = rospy.Publisher(
            "cmd_motor_rpm", Float32ListStamped, queue_size=10)
        self.tf_odom = tf2_ros.TransformBroadcaster()

    def callback_encoder(self, data):
        """
        编码器计数回调函数.

        :param encoder: [encoder]编码器计数消息
        :returns: [类型]描述
        :raises: no exception
        """
        self.drive.calculat_encoder(
            data.header.stamp.to_sec(), data.values[0], data.values[1])
        self.drive.update_odom_tf()
        self.odom_pub.publish(self.drive.odom)
        self.tf_odom.sendTransform(self.drive.tf)

    def callback_vel(self, twist):
        """
        转换转速回调函数.

        :param twist: [twist]速度指令
        :returns: [类型]描述
        :raises: no exception
        """
        _rpm = Float32ListStamped()
        _rpm.header = rospy.Header(stamp=rospy.Time.now())
        _rpm.values = [0.0, 0.0]
        self.drive.twist_to_wheel_speed(twist)
        _rpm.values[0] = self.drive.rpm_motor_l
        _rpm.values[1] = self.drive.rpm_motor_r
        self.motor_rpm_pub.publish(_rpm)


if __name__ == "__main__":
    diff_drive = DiffDriveNode()
    rospy.spin()
