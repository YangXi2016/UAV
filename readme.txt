主函数，开启多进程。

Take_off_withpid(height,mode)为起飞函数；
Patrol(set_y,timeout,signature,mode)为巡线函数；
SetFly(set_x,set_y,timeout,mode)为飞行设定函数；
Fix_Point(timeout,signature)为定点函数；
Fly()为全局飞行函数。
其余为测试函数
多进程分别包括飞行进程，串口通信进程，摄像头信息进程，其通过主函数里的key_funcion()函数里的热键来启动进程、关闭进程。

Flight_control.py
跟树莓派通讯读写的函数

Flight_Serial.py
负责底层的串口交互

CameraDetect.py
负责颜色识别和光流速度获取


摄像头相关数据处理包括滤波都转移到arduino.为树莓派节省一个进程及资源。树莓派主要负责主控即可