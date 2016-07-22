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

def handle_frame(rawFrame):
    frame_time=time.time()
    frame=rawFrame.array
    kernel=np.ones((5,5),np.uint8)
    frame=cv2.erode(frame,kernel,iterations=1)
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    #gaussion filter
    blur=cv2.GaussianBlur(gray,(5,5),0)    

    #set threshold,set a fixed threshold
    ret1,th1=cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #ret1,th1=cv2.threshold(blur,190,255,cv2.THRESH_BINARY)
    if(ret1==0):
	return None
    #cv2.imshow('th1',th1)
    lines=cv2.HoughLines(image=th1, rho=3, theta=np.pi/60, threshold=60)
    if lines==None:
	return None 
    #print 'frametime:',time.time()-frame_time
    return lines[0]

#input lines,output:return rho,theta;what's more:show img
def handle_data(lines,img_toshow):
    data_time=time.time()
    rho,theta=lines[0]
    a=np.cos(theta)
    b=np.sin(theta)
    x0=a*rho
    y0=b*rho
    x1=int(x0+1000*(-b))
    y1=int(y0+1000*(a))
    x2=int(x0-1000*(-b))
    y2=int(y0-1000*(a))

    '''cv2.line(img_toshow,(x1,y1),(x2,y2),(255,0,255),2)
    cv2.circle(img_toshow,(64,48),6,(0,0,255),4)
    cv2.circle(img_toshow,(x1,y1),3,(0,255,0),3)
    cv2.circle(img_toshow,(x2,y2),3,(255,255,0),3)
    cv2.imshow('toshow',img_toshow)'''   
    #print 'datatime',time.time()-data_time
    return rho,theta

def next_frame(rawCapture):
    if rawCapture.seekable():
	rawCapture.seek(0)
	rawCapture.truncate(0)
    else:
	rawCapture.truncate(0) 
	

#transform the axis and calculate (,angle)
#angle is directly relevant to YAW
#when out_x,out_y is negative, it means the line is left and behind
#when angle is negative, it means left,unit:radian
#if error occurs,return None
def cal_transform(rho,theta):
    cal_time=time.time()
    CENTER_X=64
    CENTER_Y=48    
    try:
	if(theta==0):
	    angle_YAW=0
	    out_y=0
	    out_x=rho-CENTER_X
	    #print 'caltime:',time.time()-cal_time
	    return out_x,out_y,angle_YAW
	elif(theta>math.pi/2):
	    angle_YAW=theta-math.pi
	else:
	    angle_YAW=theta

	x0=rho/math.cos(theta)
	y0=rho/math.sin(theta)
	k=y0/x0
	out_x=(y0+CENTER_X/k-CENTER_Y)/(k+1/k)
	out_y=-k*out_x+y0
	out_y=96-out_y
	out_x=out_x-CENTER_X
	out_y=out_y-CENTER_Y
	#print 'caltime:',time.time()-cal_time
	return out_x,out_y,angle_YAW
    except:
	print 'calculation error'
	#print 'caltime:',time.time()-cal_time
	return None,None,None
    

def Offset_Detect(offset_array):
    FRAME_WIDTH = 128
    FRAME_HEIGHT = 96    
    camera = PiCamera()
    camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)
    camera.framerate = 30

    time.sleep(2)
    #fix the values
    camera.iso=400
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g      

    rawCapture = PiRGBArray(camera, size=(FRAME_WIDTH, FRAME_HEIGHT))
    time.sleep(0.1) 

    last_time=0
    for rawFrame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	#print 'one frame start'
	#print 'all_time:',time.time()-last_time
	last_time=time.time()        

	img_toshow=rawFrame.array
	lines=handle_frame(rawFrame)
	if(lines==None):
	    print 'fail to get lines'
	    offset_array[0]=255
	    next_frame(rawCapture)
	    continue
	#print 'innerlinestime:',time.time()-last_time

	rho,theta=handle_data(lines,img_toshow)
	#print rho,theta
	out_x,out_y,angle_YAW=cal_transform(rho, theta)
	if(out_x==None):
	    print 'fail to get output'
	    offset_array[0]=255
	    next_frame(rawCapture)
	    continue
	#print out_x,out_y,angle_YAW
	#print 'outputtime:',time.time()-last_time
	offset_array[:]=[out_x,out_y,angle_YAW]
	next_frame(rawCapture)    

	#rawCapture.truncate(0)
	#print 'seektime:',time.time()-last_time
	k=cv2.waitKey(1)&0xFF  
	if k==27:
	    break 
	#print 'keytime:',time.time()-last_time
    camera.close()
    cv2.destroyAllWindows()