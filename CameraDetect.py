# -*- coding: utf-8 -*-
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial
import math
import copy
from define import *


# 图像大小
FRAME_WIDTH = 160#120
FRAME_HEIGHT =120#90

# HSV阈值范围
HUE_BLUE = 240/2
HUE_YELLOW = 60/2
HUE_RED = 0
HUE_RANGE = 25#15

SAT_MIN = 100
SAT_MAX = 255

VAL_MIN = 50
VAL_MAX = 255

# 形态学操作范围
CLOSE_MASK_SIZE = 30
KERNEL = np.ones((CLOSE_MASK_SIZE, CLOSE_MASK_SIZE), np.uint8)
#右倾ROL变大；后倾PIT变大；顺时针YAW变大
#ROL变大右倾；PIT变大qian倾；YAW变大逆时针
#(6/14之后废除)得到标志物的坐标（x,y）：x为正代表标志物在飞机左方,对应飞机的ROL量应该为正，使得飞机向右飞行，即发送过去的ROL应增加，；y为正代表标志物在飞机前方，对应飞机的PIT量应该为正，使得飞机向后飞行，即发送过去的PIT应增加；
#(为了使飞机笔直飞行，打算再用一条条彩带标志方向，通过霍夫检测直线，得到其斜率，然后矫正到一致。顺时针YAW变大；逆时针YAW变小；【貌似没必要了，直接保证YAW值保持初始值不变即可；或者初始飞机的目标前进方向对应的YAW就是0，保持YAW动态稳定在零。】)

#光流法下：#摄像头向后运动，y值为正；摄像头向右运动，x值为正。

#ROL变大右倾
# u:像素距离
# depth 真实深度
def getRealLength(u, depth):
    return depth * 0.447*2*1.09*u/24 # 这个公式根据三角关系得知(depth单位为cm)


def processImage(hsv, color, frame):
    #根据颜色选择不同的HSV阈值范围 红色要特殊处理
    if color == 1: #red
        lo_red1 = np.array([ HUE_RED, SAT_MIN, VAL_MIN ])
        hi_red1 = np.array([ HUE_RED + HUE_RANGE/2, SAT_MAX, VAL_MAX ])
        lo_red2 = np.array([ 180 - HUE_RANGE/2, SAT_MIN, VAL_MIN ])
        hi_red2 = np.array([ 180, SAT_MAX, VAL_MAX ])
        mask1 = cv2.inRange(hsv, lo_red1, hi_red1)
        mask2 = cv2.inRange(hsv, lo_red2, hi_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        circleColor = (0, 0, 255)
    elif color == 2: # blue
        lower_blue = np.array([ HUE_BLUE - HUE_RANGE, SAT_MIN, VAL_MIN ])
        upper_blue = np.array([ HUE_BLUE + HUE_RANGE, SAT_MAX, VAL_MAX ])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        circleColor = (255, 0, 0)
    elif color == 3: #yellow
        lower_y = np.array([ HUE_YELLOW - HUE_RANGE, SAT_MIN, VAL_MIN ])
        upper_y = np.array([ HUE_YELLOW + HUE_RANGE, SAT_MAX, VAL_MAX ])
        mask = cv2.inRange(hsv, lower_y, upper_y)
        circleColor = (0, 255, 255)
    #形态学滤波 先闭后开
    cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)

    # 轮廓检出
    ctr = mask.copy()
    contours, hierarchy = cv2.findContours(ctr,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x=0
    y=0
    radius=0
    contour=None
    for temp_contour in contours:
        # 查找外接圆
        (temp_x,temp_y),temp_radius = cv2.minEnclosingCircle(temp_contour)
        if temp_radius>radius:
            radius=temp_radius
            x=temp_x
            y=temp_y
            contour=temp_contour
    #(realX, realY) = getDepthAndAngle(radius, color, x, y,depth)
    cv2.drawContours(frame, contour, -1, circleColor, 1)
    cv2.circle(frame, (int(x),int(y)), int(radius), circleColor, 2)
    '''
        realX = '%d'%int(realX)
        realY = '%d'%int(realY)
        str_radius= '%d'%int(radius)
        serData = '   ' + realX+ ' ' + realY + ' ' + str_radius
        ser.write(serData)
        print serData
        '''
    dy = y - FRAME_HEIGHT/2 
    dx = x - FRAME_WIDTH/2 
    return dx,dy,radius
#改用Arduino+摄像头模块
CAMERA_SER=serial.Serial(CAMERA_COM, 115200,timeout=0.2)
line_offset=0
speed_x=0
speed_y=0
object_x=0
object_y=0
def camera_info():
    CAMERA_SER.flushInput()
    CAMERA_SER.write('i')
    data=CAMERA_SER.read(8)
    result=[0,0,0,0,0,0,0,0]
    if(len(data)!=8):
        print "camera_info error"
    else:
        for i in range(8):
            if ord(data[i])==0:
                result[i]=0
            else:
                result[i]=ord(data[i])-127
    #return [line_offset,object_x,object_y,speed_x,speed_y]
    return result

def Offset_Detect(offset_data_queue):
    data=[0,0,0,0,0]
    old_data=camera_info() 
    '''filepath1=time.strftime( '%Y-%m-%d %X', time.localtime() )
    filepath2=filepath1
    filepath1+='sourcedata.txt'
    filepath2+='resultdata.txt'''
    
    while(1):
	#file1=open(filepath1,'a')
	#file2=open(filepath2,'a')
	new_data=camera_info()
	for i in range(5):
	    data[i]=int(0.9*old_data[i]+0.1*new_data[i])
	    '''file1.write(str(new_data[i])+'  ')
	    file2.write(str(data[i])+'  ')
	file1.write('\r\n')
	file2.write('\r\n')
	file1.close()
	file2.close()'''
	#print data
	
	old_data=data
        safe_put(offset_data_queue, data)

	
    '''with PiCamera() as camera:
        #初始化相机 图像大小\采集速率
        #camera = PiCamera()
        camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)
        camera.awb_mode = "incandescent"
        camera.iso=0
        camera.framerate = 10
        rawCapture = PiRGBArray(camera, size=(FRAME_WIDTH, FRAME_HEIGHT))
        time.sleep(0.1)

        preColor = None #光流所用
        # 以BGR格式采集图像
        for rawFrame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            frame = rawFrame.array
            # 颜色空间变换
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 图像处理
            offset_data=processImage(hsv, 1, frame)
            safe_put(offset_data_queue, offset_data)
            processImage(hsv, 2, frame)
            processImage(hsv, 3, frame)

    ##      cv2.imshow('mask',mask)
            cv2.imshow("OffsetDetect", frame)
            
            # q键退出
            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            if key == ord("q"):
                break

        cv2.destroyAllWindows()'''
