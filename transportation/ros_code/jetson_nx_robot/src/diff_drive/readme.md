# diff_drive
## Nodes
- diff_drive
  
  差速驱动计算节点
## Subscribed Topics
- motor_encoder (custom_msgs/Float32ListStamped)

  电机编码器数据
- cmd_vel (geometry_msgs/Twist)

  线速度、角速度控制指令
## Published Topics
- cmd_motor_rpm (custom_msgs/Float32ListStamped)

  电机转速控制指令
- odom_drive (nav_msgs/Odometry)

  odom数据
- tf  (tf/tfMessage)
  
  发布base_link到odom的tf信息
  
## Services
- xxx

## Parameters
- diff_drive/wheel_radius (float,default: 0.15/2)
  
  轮胎半径
- diff_drive/wheel_base (float,default: 0.53)
  
  两轮中心距
- diff_drive/resolution (int,default: 10000)
  
  编码器分辨率
- diff_drive/count_bits (int,default: 32)
  
  编码器计数位数，4字节：32位
- diff_drive/gear_ratio (float,default: 20.0)
  
  减速箱速比
- diff_drive/reverse_l (bool,default: False)
  
  左轮旋转是否反向
- diff_drive/reverse_r (bool,default: True)
  
  右轮旋转是否反向
- diff_drive/time_out (float,default: 3.0)
  
  电机数据超时时间
