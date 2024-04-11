# light_ctrl
## Nodes
- light_ctrl
  
  灯光控制节点
## Subscribed Topics
- light (std_msgs/ColorRGBA)
  
  灯颜色，目前只支持开关量，颜色通道>0即开
- light/turn_l (std_msgs/Bool)
  
  左转向灯
- light/turn_r (std_msgs/Bool)
  
  右转向灯
## Published Topics
- xxx
## Services
- xxx

## Parameters
- light/pin_r (int,default: 33)
  
  GPIO红色led管脚
- light/pin_g (int,default: 32)
  
  GPIO绿色led管脚
- light/pin_b (int,default: 15)
  
  GPIO蓝色led管脚
- light/pin_turn_l (int,default: 16)
  
  GPIO左转向灯管脚
- light/pin_turn_r (int,default: 18)
  
  GPIO右转向灯管脚

