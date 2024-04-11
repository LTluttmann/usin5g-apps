# lid_ctrl
## Nodes
- lid_ctrl
  
  开关盖控制节点
## Subscribed Topics
- xxx
## Published Topics
- lid/unclosed (std_msgs/Bool)
  
  盖子未关状态
## Services
- lid_ctrl (std_srvs/SetBool)
  
  开关盖控制 True:开盖 False:关盖

## Parameters
- lid/pin_open (int,default: 11)
  
  GPIO开盖管脚
- lid/pin_close (int,default: 12)
  
  GPIO关盖管脚
- lid/pin_unclosed (int,default: 13)
  
  GPIO盖子未关输入管脚
- lid/time_open (float,default: 3.0)
  
  开盖时间，秒
- lid/time_close (float,default: 3.0)
  
  关盖时间，秒
