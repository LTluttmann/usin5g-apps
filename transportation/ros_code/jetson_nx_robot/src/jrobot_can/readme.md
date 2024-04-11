# jrobot_can
## Nodes
- jrobot_can
  
  极创底盘can通讯
## Subscribed Topics
- cmd_motor_rpm (custom_msgs/Float32ListStamped)

  电机转速控制指令
## Published Topics
- motor_encoder (custom_msgs/Float32ListStamped)

  电机编码器数据
## Services
- xxx

## Parameters
- motor_can/channel (string,default: 'can0')
  
  CAN通道
- motor_can/bitrate (int,default: 250000)
  
  通讯速率
- motor_can/id (int,default: 0x601)
  
  底盘CAN ID，默认为0x601
- motor_can/rate_encoder (int,default: 20)
  
  编码器读取频率，默认20hz

