# 使用
启动模拟电机节点
```
ros2 run motor_simulator motor_simulator_node    
```
运行滤波器节点，中值滤波和低通滤波器两个话题，这里默认用低通滤波器处理电机的返回数据
```
ros2 run third filter_node
```
运行pid控制节点，订阅低通滤波器话题，使用其数据进行电机pid控制，有速度控制和角度控制，由于这两个功能在一定程度上会冲突，因此使用一个功能时要注释掉另一个功能，具体操作在test_ws/src/motor_simulator-ros2/src/pid_control.cpp里有标注
```
ros2 run motor_simulator pid_control
```
运行噪声信号生成器节点
```
ros2 run third signal_generate_third
```