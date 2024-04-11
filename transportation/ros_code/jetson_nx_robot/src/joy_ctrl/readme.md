# joy_ctrl
## Nodes
- joy_ctrl
  
  手柄控制，功能：开关盖，开关灯，左右转向灯
## Subscribed Topics
- joy (sensor_msgs/Joy)

  手柄状态
## Published Topics
- light (std_msgs/ColorRGBA)

  灯光控制
- light/turn_l (std_msgs/Bool)
  
  左转向灯
- light/turn_r (std_msgs/Bool)
  
  右转向灯
## Services
- xxx

## Parameters
- joy/axes_lid (int,default: 1)
  
  手柄控制开关盖轴，默认为上下键
- joy/button_light_r (int,default: 1)
  
  手柄控制红灯键
- joy/button_light_g (int,default: 0)
  
  手柄控制绿灯键
- joy/button_light_b (int,default: 2)
  
  手柄控制蓝灯键
- joy/axes_light_turn (int,default: 6)
  
  手柄控制左右转向灯，默认为左右键
