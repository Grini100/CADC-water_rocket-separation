# CADC-water_rocket-separation
- 硬件需求：STM32F103、OLED、MPU6050、W25Q64、按键x2、舵机x2
- 软件功能：设置多重状态机，以及状态转换条件，完成水火箭与滑翔机分离和开伞功能
- analyzer.py：分析水火箭飞行数据，生成飞行轨迹图和高度估算图;用法：python analyzer.py  飞行数据.bin
- 飞行数据储存在W25Q64中，可通过串口打印到PC端
## 具体详情见doc文件夹中文件
