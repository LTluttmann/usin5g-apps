# ultrasonic
## Nodes
- ultrasonic
  
  超声波传感器节点
## Subscribed Topics
- ultrasonic_data (custom_msgs/Float32ListStamped)

  超声波传感器数据
## Published Topics
- ultrasonic/FC (sensor_msgs/Range)

  超声波传感器-前中
- ultrasonic/RC (sensor_msgs/Range)

  超声波传感器-后中
- ultrasonic/LC (sensor_msgs/Range)

  超声波传感器-左中
- ultrasonic/RC (sensor_msgs/Range)

  超声波传感器-右中
- ultrasonic/FL (sensor_msgs/Range)

  超声波传感器-左前
- ultrasonic/FR (sensor_msgs/Range)

  超声波传感器-右前
- ultrasonic/RL (sensor_msgs/Range)

  超声波传感器-左后
- ultrasonic/RR (sensor_msgs/Range)

  超声波传感器-右后
## Services
- xxx

## Parameters
- ultrasonic/field_of_view(float,default: 60.0)
  
  超声波传感器FOV,默认：60度
- ultrasonic/min_range(float,default: 0.28)
  
  超声波传感器最小距离,默认：0.28m
- ultrasonic/max_range(float,default: 4.5)
  
  超声波传感器最大距离,默认：4.5m
- ultrasonic/data_index(list,default: [])
  
  超声波传感器数据位置索引，[前，后，左，右]


