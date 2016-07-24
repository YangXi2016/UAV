# -*- coding: utf-8 -*
import multiprocessing as mp
import time
from Tkinter import *

#发送头 list
HEAD_SEND=[0xAA,0xAF]   #tuple不可变
HEAD_RECE=[0xAA,0xAA]


#功能字 hex
COMMAND=0x01
RCDATA=0x03
CHECK=0xEF
REQUEST=0xF1
TYPE_END=0xFA
#命令内容 list
cmd_unlock=[0xA1]
cmd_lock=[0xA0]
cmd_baroheight_mode=[0xA2]
cmd_ultraheight_mode=[0xA3]
cmd_unheight_mode=[0xA4]
request_data=[0x00]
#11111111 八位分别代表请求 （ NA NA [飞机传感器数据]  收到的控制数据 马达PWM(范围0-1000) 速度信息 高度信息  用户自定义信息）

#要发送的rc_data[int16 THR int16 YAW int16 ROL nt16 PIT int16 AUX1 int16 AUX2 int16 AUX3 int16 AUX4 int16 AUX5 int16 AUX6]
RC_DATA_NUM=10
rc_data=[1500 for i in range(10)]
rc_data[0]=1100

#pid参数
#定点悬停pid参数
'''

kp_height-0
ki_height=0
kd_height=0

kp_yaw=0
ki_yaw=0
kd_yaw=0

kp_rol=0#0.2#2
ki_rol=0
kd_rol=0

kp_pit=0
ki_pit=0
kd_pit=0

'''


#右倾ROL变大；后倾PIT变大；顺时针YAW变大
#ROL变大右倾；PIT变大qian倾；YAW变大逆时针
#(6/14之后废除)得到标志物的坐标（x,y）：x为正代表标志物在飞机左方,对应飞机的ROL量应该为正，使得飞机向右飞行，即发送过去的ROL应增加，；y为正代表标志物在飞机前方，对应飞机的PIT量应该为正，使得飞机向后飞行，即发送过去的PIT应增加；
#(为了使飞机笔直飞行，打算再用一条条彩带标志方向，通过霍夫检测直线，得到其斜率，然后矫正到一致。顺时针YAW变大；逆时针YAW变小；【貌似没必要了，直接保证YAW值保持初始值不变即可；或者初始飞机的目标前进方向对应的YAW就是0，保持YAW动态稳定在零。】)

#光流法下：#摄像头向后运动，y值为正；摄像头向右运动，x值为正。

#height,rotate,left/right,ahead/back,speed_x,speed_y
kp_x=0.12
kp_y=0.1
'''kp=[0,0,0,0,0,0]
ki=[0,0,0,0,0,0]
kd=[0,0,0,0,0,0]'''
'''kp=[0,0, 4,  -4,    0,0]
ki=[0,0, 4,  -4,   0,0]
kd=[0,0, 0,       0,   0,0]'''
kp=[0,0, 4,    -3.7,    0,0]
ki=[0,0, 1.2,  -1,   0,0]
kd=[0,0, 0,    0,   0,0]
'''kp=[0,0.1,1,1,0,0]
ki=[0,0.03,0,0,0,0]
kd=[0,0.01,0.01,0.01,0,0]'''
position=[0,0,0,0]	    #位置外环控制
position_times=5
position_i=0 

OFFSET=[1500,1500,1520,1540]#[thr_offset,yaw_offset,rol_offset,pit_offset]
RANGE=[300,100,100,100]#[thr_range,yaw_range,rol_range,pit_range]
rc_data[1:4]=OFFSET[1:4]


#数据交换通道
#senser_queue = mp.Queue(1)   #senser_data传输的通道（数据从Serial_Monitor传出来）
#data_queue = mp.Queue(1)    #status_data传输的通道（数据从Serial_Monitor传出来）
#out_queue = mp.Queue(1)     #request_cmd传输的通道（数据传到Serial_Monitor）
#offset_data_queue=mp.Queue(1)   #request_data传输的通道（数据从Serial_Monitor传出来）
senser_array=mp.Array('B', [0 for i in range(12)])
data_array=mp.Array('B', [0 for i in range(14)])
out_array =mp.Array('B', [0 for i in range(30)])
offset_array=mp.Array('i',[0 for i in range(9)])
#print data_array
#print out_array

goal_height=70
#初始化时从飞控得到的初始状态
HEIGHT_INIT=None
YAW_INIT=None
#SPEED_X_INIT=0
#SPEED_Y_INIT=0
SPEED_LIMIT=15

PIN_CTR=29
PIN_ERR=31


#选择串口
#SER_COM='COM5'
SER_COM='/dev/ttyUSB0'
CAMERA_COM='/dev/ttyUSB1'
#safe_get and saft_put

'''def safe_put(queue,data):
    last_time=time.time()
    while(time.time()-last_time<1):
        try:
            if(queue.empty()==False):
                queue.get(FALSE)
            queue.put(data,False)
            break
        except:         
            pass    

def safe_get(queue):
    last_time=time.time()
    while(time.time()-last_time<2):
        if(queue.empty()==FALSE):
            try:
                data=queue.get(False)
                return data           
            except:
                pass
                #print("safe_get error")
        time.sleep(0.05)
    return 0
'''

#GUI var
'''master = Tk()

scrollbar = Scrollbar(master)

scrollbar.pack(side=RIGHT, fill=Y)

listbox = Listbox(master, yscrollcommand=scrollbar.set)
#listbox["width"]=50
#listbox["height"]=80
#listbox.see(END)    
listbox.pack(side=LEFT, fill=BOTH)

scrollbar.config(command=listbox.yview)'''

filename=time.strftime( '%Y-%m-%d %X', time.localtime() )
filename+='.txt'
filehanher=open(filename, mode='a')