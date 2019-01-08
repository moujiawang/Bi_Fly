
# 协议

## 下行

### 执行器控制类型

报文ID: 0xA0
|控制量 | 数据类型|
|---|---|
|拍打电机转速|uint8_t|
|爬行电机转速|uint8_t|
|俯仰舵机| int8_t|
|横滚舵机| int8_t|
|偏航舵机| int8_t|

### 运动控制类型
报文ID: 0xA0
控制量 | 数据类型
---|---
前后|uint8_t
左右|uint8_t
升降|uint8_t
偏航|uint8_t

### 修改PID参数
报文ID: 0xA1
控制量 | 数据类型
---|---
控制环ID|uint16_t
Kp|uint16_t
Ki|uint16_t
Kd|uint16_t



## 上行
### 基础控制类型
报文ID: 0xA0
控制量 | 数据类型
---|---
电机转速|uint8_t
俯仰舵机|uint8_t
横滚舵机|uint8_t
偏航舵机|uint8_t

报文ID: 0xA0
控制量 | 数据类型
---|---
俯仰角|float
横滚角|float
偏航角|float
x坐标|float
y坐标|float
z坐标|float


