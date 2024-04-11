#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
-------------------------------------------------------------------------------
File Name : joy_navi.py
Description : 手柄控制导航
Author : zhangxu
Create Time : 2022-11-24
-------------------------------------------------------------------------------
Change Activity:
        zhangxu:更改内容
-------------------------------------------------------------------------------
"""
__author__ = 'zhangxu'


from datetime import datetime
import os
import time
import rospy
import rosnode
import subprocess
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState


enable_button_navi = rospy.get_param("joy/enable_button_navi", 5)
button_global_localization = rospy.get_param(
    "joy/button_global_localization", 3)
button_clear_costmaps = rospy.get_param("joy/button_clear_costmaps", 2)
button_start_cartographer = rospy.get_param("joy/button_start_cartographer", 1)
button_save_cartographer = rospy.get_param("joy/button_save_cartographer", 0)
axes_change_map = rospy.get_param("joy/axes_change_map", 0)
map_dir = rospy.get_param(
    "map_dir", "/home/ros/ros_code/jetson_nx_robot/user/maps/")
map_dir_cartographer = map_dir+"cartographer/"

SH_START_CARTOGRAPHER = "roslaunch navi_ctrl cartographer_backpack_2d.launch"
# rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=$map_dir/$map_name.pbstream  -map_filestem=$map_dir/$map_name
SH_PBSTREAM_TO_ROS_MAP = "rosrun cartographer_ros cartographer_pbstream_to_ros_map"
PBSTREAM_PARAM_1 = "pbstream_filename"
PBSTREAM_PARAM_2 = "map_filestem"
# rosrun map_server map_saver map:=/<your map topic> -f <map_name>
SH_MAP_SAVER = "rosrun map_server map_saver map:=/map -f"
# rosrun map_server map_server mymap.yaml
SH_MAP_SERVER = "rosrun map_server map_server __name:=map_server"
SH_START_AMCL = "roslaunch navi_ctrl move_base_amcl.launch"

SOURCE_ROS = "source /opt/ros/melodic/setup.bash"
SOURCE_WS = "source /home/ros/ros_code/jetson_nx_robot/devel/setup.bash"
SOURCE_RSLIDAR = "source /home/ros/ros_code/rslidar_ws/devel/setup.bash"
SOURCE_CARTOGRAPHER = "source /home/ros/ros_code/cartographer_ws/install_isolated/setup.bash"

SOURCE = SOURCE_ROS + " && " + SOURCE_WS + " && " + SOURCE_CARTOGRAPHER


def amcl_global_localization():
    """amcl重定位."""
    print("amcl重定位")
    rospy.wait_for_service("/global_localization")
    try:
        srv_global_localization = rospy.ServiceProxy(
            "/global_localization", Empty)
        resp = srv_global_localization()
    except rospy.ServiceException as e:
        print(e)


def clear_costmaps():
    """清除costmaps."""
    print("清除costmaps")
    rospy.wait_for_service("/move_base_node/clear_costmaps")
    try:
        srv_clear_costmaps = rospy.ServiceProxy(
            "/move_base_node/clear_costmaps", Empty)
        resp = srv_clear_costmaps()
    except rospy.ServiceException as e:
        print(e)


def kill_cartographer():
    """结束建图进程."""
    rosnode.kill_nodes(["cartographer_node"])
    rosnode.kill_nodes(["cartographer_occupancy_grid_node"])


def kill_map_server():
    """结束map_server."""
    rosnode.kill_nodes(["map_server"])


def kill_amcl():
    """结束amcl."""
    rosnode.kill_nodes(["move_base_node", "amcl"])


def start_amcl():
    """结束amcl."""
    subprocess.Popen(SH_START_AMCL, shell=True, stdout=subprocess.PIPE,
                     stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def start_cartographer():
    """开始建图."""
    subprocess.Popen(SH_START_CARTOGRAPHER, shell=True, stdout=subprocess.PIPE,
                     stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def end_cartographer():
    """结束建图."""
    rospy.wait_for_service("/finish_trajectory")
    try:
        srv_end_cartographer = rospy.ServiceProxy(
            "/finish_trajectory", FinishTrajectory)
        resp = srv_end_cartographer(0)
        rospy.loginfo(resp.status)
        print("finish_trajectory")
    except rospy.ServiceException as e:
        print(e)


def save_cartographer(file_name):
    """保存建图."""
    rospy.wait_for_service("/write_state")
    try:
        srv_save_cartographer = rospy.ServiceProxy(
            "/write_state", WriteState)
        resp = srv_save_cartographer(
            filename=file_name+".pbstream")
        print("保存建图", file_name+".pbstream")
        rospy.loginfo(resp.status)
    except rospy.ServiceException as e:
        print(e)


def cartographer_to_ros_map(pbstream_filename, map_filestem):
    """地图转换为ros格式."""
    subprocess.Popen(f"{SH_PBSTREAM_TO_ROS_MAP} -{PBSTREAM_PARAM_1}={pbstream_filename} -{PBSTREAM_PARAM_2}={map_filestem}", shell=True, stdout=subprocess.PIPE,
                     stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def save_ros_map(file):
    """mapserver保存地图."""
    #os.system(f"{SH_MAP_SAVER} {file}")
    subprocess.Popen(f"{SH_MAP_SAVER} {file}", shell=True, stdout=subprocess.PIPE,
                     stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def change_map(file_name):
    """mapserver切换地图."""
    kill_cartographer()
    kill_map_server()
    kill_amcl()
    # time.sleep(3)
    subprocess.Popen(f"{SH_MAP_SERVER} {file_name}.yaml", shell=True, stdout=subprocess.PIPE,
                     stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(5)
    start_amcl()
    print("地图切换完成")


def start_build_map():
    """开始建图."""
    print("开始建图")
    kill_amcl()
    kill_map_server()
    kill_cartographer()
    start_cartographer()
    print("建图已启动")


def end_build_map():
    """结束建图，应用新地图."""
    print("结束建图")
    _now = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    _map_name = map_dir+_now
    _map_cartographer = map_dir_cartographer+_now
    print(_map_name, _map_cartographer)
    end_cartographer()
    time.sleep(5)
    save_cartographer(_map_cartographer)
    cartographer_to_ros_map(f"{_map_cartographer}.pbstream", _map_name)
    # save_ros_map(_map_name)
    kill_cartographer()
    time.sleep(5)
    change_map(_map_name)
    print("建图已结束")


def get_all_maps_name(dir):
    """
    获取目录中所有地图名.

    :param dir: [str]目录
    :returns: [list]包含所有地图名的列表，时间最晚的在最前
    :raises: no exception
    """
    _map_names = []
    _files = os.listdir(dir)
    for i in _files:
        if len(i) > 5 and i[-5:] == ".yaml":
            _map_names.append(i[:-5])
    _map_names.sort(reverse=True)
    # print(_map_names)
    return _map_names


class JoyNaviNode:
    def __init__(self):
        rospy.init_node("joy_navi")
        rospy.loginfo("Starting JoyNaviNode as joy_navi.")

        self.joy_date_lase = None
        rospy.Subscriber("joy", Joy, self.callback_joy)

        self.map_index = 0
        # _maps_list = get_all_maps_name(map_dir)
        # subprocess.Popen(f"{SH_MAP_SERVER} {_maps_list[self.map_index]}.yaml", shell=True, stdout=subprocess.PIPE,
        #                  stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def callback_joy(self, data):
        """
        手柄回调.

        :param data: [Joy]手柄值
        :returns: [类型]描述
        :raises: no exception
        """
        if self.joy_date_lase is None:
            self.joy_date_lase = data
            return None
        if data.buttons[enable_button_navi]:
            # 重定位
            if self.joy_date_lase.buttons[button_global_localization] == 0 and data.buttons[button_global_localization] == 1:
                amcl_global_localization()

            # 清除costmap
            if self.joy_date_lase.buttons[button_clear_costmaps] == 0 and data.buttons[button_clear_costmaps] == 1:
                clear_costmaps()

            # 开始建图
            if self.joy_date_lase.buttons[button_start_cartographer] == 0 and data.buttons[button_start_cartographer] == 1:
                start_build_map()
            # 结束建图
            if self.joy_date_lase.buttons[button_save_cartographer] == 0 and data.buttons[button_save_cartographer] == 1:
                end_build_map()
                self.map_index = 0
            # 切换下一个地图
            if self.joy_date_lase.axes[axes_change_map] == 0 and data.axes[button_save_cartographer] == -1:
                _maps_list = get_all_maps_name(map_dir)
                if self.map_index < len(_maps_list)-1:
                    self.map_index = self.map_index+1
                    print(
                        f"切换地图 index:{self.map_index} name:{_maps_list[self.map_index]}")
                    change_map(map_dir+_maps_list[self.map_index])

                else:
                    pass
            # 切换上一个地图
            if self.joy_date_lase.axes[axes_change_map] == 0 and data.axes[button_save_cartographer] == 1:
                _maps_list = get_all_maps_name(map_dir)
                if self.map_index > 0:
                    self.map_index = self.map_index-1
                    print(
                        f"切换地图 index:{self.map_index} name:{_maps_list[self.map_index]}")
                    change_map(map_dir+_maps_list[self.map_index])

                else:
                    pass
        else:
            pass
        self.joy_date_lase = data


if __name__ == "__main__":
    joy_ctrl = JoyNaviNode()
    rospy.spin()
