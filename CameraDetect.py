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
import RPi.GPIO as GPIO

# 图像大小
FRAME_WIDTH = 160#120
FRAME_HEIGHT =120#90

# HSV阈值范围
HUE_BLUE = 190/2
HUE_YELLOW = 60/2
HUE_RED = 0
HUE_RANGE = 5#15

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


#改用Arduino+摄像头模块
CAMERA_SER=serial.Serial(CAMERA_COM, 115200,timeout=0.4)
line_offset=0
speed_x=0
speed_y=0
object_x=0
object_y=0
def camera_info():
    #global SPEED_X_INIT,SPEED_Y_INIT
    CAMERA_SER.flushInput()
    CAMERA_SER.write('i')
    data=CAMERA_SER.read(8)
    time.sleep(0.005)
    result=[0,0,0,0,0,0,0,0]
    if(len(data)!=8):
        print "camera_info error"
	GPIO.output(PIN_ERR,GPIO.HIGH)
    else:
	GPIO.output(PIN_ERR,GPIO.LOW)
        for i in range(6):
                result[i]=ord(data[i])
	result[6]=127-ord(data[6])#-SPEED_X_INIT
	result[7]=127-ord(data[7])#-SPEED_Y_INIT
    #return [line_offset,object_x,object_y,speed_x,speed_y]
    print result
    return result

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
    elif color == 3: # blue
	lower_blue = np.array([ HUE_BLUE - HUE_RANGE, SAT_MIN, VAL_MIN ])
	upper_blue = np.array([ HUE_BLUE + HUE_RANGE, SAT_MAX, VAL_MAX ])
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	circleColor = (255, 0, 0)
    elif color == 2: #yellow
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
    if(radius>4):
	dy =int(y - 90/2 - 7)#为了解决投球装置滞后引入的偏执
	dx =int(x - 120/2)
	radius=int(radius)
	#print dy,dx,radius
    else:
	dy=-45
	dx=-60
	radius=0
	#print offset_data[color*3-3]
    return dx,dy,radius

def calibration(dis_frame):
    cameraMatrix=np.array([[133.77965, 0, 179.13740],
                           [0, 133.75846, 111.34959],
                           [0, 0, 1]])
    distCoeffs=np.array([-0.31482, 0.10522, 0.000387, 0.00001367, -0.01685])
    output_frame = cv2.undistort(dis_frame, cameraMatrix,distCoeffs,None)
    return output_frame

def next_frame(rawCapture):
    if rawCapture.seekable():
	rawCapture.seek(0)
	rawCapture.truncate(0)
    else:
	rawCapture.truncate(0) 


def Offset_Detect(offset_array):
    data=[0,0,0,0,0,0,0,0,0]	#三个一组 dx,dy,radius
    #global offset_array
    FRAME_WIDTH = 320
    FRAME_HEIGHT = 240

    HUE_BLUE = 240/2
    HUE_YELLOW = 60/2
    HUE_RED = 0
    HUE_RANGE = 10

    SAT_MIN = 100
    SAT_MAX = 255

    VAL_MIN = 50
    VAL_MAX = 255

    CLOSE_MASK_SIZE = 30
    KERNEL = np.ones((CLOSE_MASK_SIZE, CLOSE_MASK_SIZE), np.uint8)


    camera = PiCamera()
    camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)
    camera.framerate = 30
    camera.awb_mode = "incandescent"
    rawCapture = PiRGBArray(camera, size=(FRAME_WIDTH, FRAME_HEIGHT))
    time.sleep(0.1)

    last_time=0
    for rawFrame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	#print 'alltime:',time.time()-last_time
	#last_time=time.time()

	input_frame = rawFrame.array
	correct_frame = calibration(input_frame)
	frame = cv2.resize(correct_frame, (120,90))

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#print '1time:',time.time()-last_time
	data[0:3]=processImage(hsv, 1, frame)
	#print '2time:',time.time()-last_time
	data[3:6]=processImage(hsv, 2, frame)
	#print '3time:',time.time()-last_time
	data[6:9]=processImage(hsv, 3, frame)
	#print '4time:',time.time()-last_time
	#print data
	offset_array[:]=data
	#print offset_data

	cv2.imshow("Frame", frame)
	next_frame(rawCapture)
	key=cv2.waitKey(1)&0xFF
	if key == 27:
	    break

    cv2.destroyAllWindows()