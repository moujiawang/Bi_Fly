
# 协议
下行的协议分为三个模式：手动控制模式，实际应用模式，调参实验模式
三种不同的模式，所下发的指令不一样；具体根据飞行器收到的报文ID作判断；

## 下行

### 手动控制模式

报文ID: 0xA0
|控制量 | 数据类型|
|---|---|
|拍打电机转速|uint8_t|
|爬行电机转速|uint8_t|
|俯仰舵机| int8_t|
|横滚舵机| int8_t|
|偏航舵机| int8_t|

### 实际应用模式
报文ID: 0xA1
控制量 | 数据类型
---|---
前后|uint8_t
左右|uint8_t
升降|uint8_t
偏航|uint8_t

### 调参实验模式
报文ID: 0xA2

控制量 | 数据类型
---|---
PID_ID|uint8_t
Kp_Int|uint8_t
Ki_Int|uint8_t
Kd_Int|uint8_t
Kp_Ext|uint8_t
Ki_Ext|uint8_t
Kd_Ext|uint8_t
SetValue|uint16_t

PID_ID指定当前整定的是哪一个姿态角的PID,PID_ID可以为0x00,0x01,0x02;分别对应Yaw,Pitch,Roll;




## 上行
### 基础控制类型
报文ID: 0xA0
控制量 | 数据类型
---|---
电机转速|uint8_t
俯仰舵机|uint8_t
横滚舵机|uint8_t
偏航舵机|uint8_t
俯仰角|float
横滚角|float
偏航角|float
Kp_Int|uint8_t
Ki_Int|uint8_t
Kd_Int|uint8_t
Kp_Ext|uint8_t
Ki_Ext|uint8_t
Kd_Ext|uint8_t


