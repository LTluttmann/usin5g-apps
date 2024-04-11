#!/bin/bash
# enter the workspace
source install_isolated/setup.bash

# map  folder 
map_dir="${HOME}/catkin_ws/map"      
 # map name defined as current datatime   
map_name=$(date +%Y%m%d_%H-%M-%S)  

# check the file folder exist
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 即将创建文件夹"
  mkdir -p $map_dir
fi

# finish slam
rosservice call /finish_trajectory 0

# make pbstream
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

# pbstream to map
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name
