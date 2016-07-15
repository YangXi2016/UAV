# -*- coding: utf-8 -*-
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial
import math

# 根据像素距离和真实深度计算横向长度
# u:像素距离
# depth 真实深度
def getRealLength(u, depth):
        return depth * 0.447*2*1.09*u/120 # 这个公式根据三角关系得知

# 根据颜色 外接圆半径(像素) 圆心坐标(像素) 计算深度和物体中心的真实坐标(单位mm)
def getDepthAndAngle(radius, color, x, y):
        if color == 1:          # red
                realSize = 353         # 250mm * sqrt(2)
        elif color == 2:        # blue
                realSize = 565         # 400mm * 1.414
        elif color == 3:
                realSize = 848         # 600 * 1.414

        depth =  realSize / radius *61.5 # 这个系数根据比例关系计算得知 详见excel
        dy = FRAME_HEIGHT/2 - y
        dx = x - FRAME_WIDTH/2

        realX = getRealLength(dx, depth)
        realY = getRealLength(dy, depth)
        return depth, (realX, realY)
# 处理图像,查找对应的颜色区域,从串口输出
# hsv: HSV空间的图像
# color : 需要检测的颜色
# frame : 原始图像
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

        for contour in contours:
                # 查找外接圆
                (x,y),radius = cv2.minEnclosingCircle(contour)
                # 只查找半径大于10的色块
                if radius > 10:
                        center = (int(x),int(y))
                        radius = int(radius)
                        # 计算真实坐标 单位mm
                        depth, (realX, realY) = getDepthAndAngle(radius, color, x, y)
                        cv2.drawContours(frame, contours, -1, circleColor, 1)
                        cv2.circle(frame, center, radius, circleColor, 2)

                        # 串口输出 屏幕输出
                        depth = '%d'%int(depth)
                        realX = '%d'%int(realX)
                        realY = '%d'%int(realY)
                        colorTemp = '%d'%color
                        serData = '   ' + colorTemp + ' ' + depth + ' ' + realX + ' ' + realY

                        #ser.write(serData)
                        print serData
def hough(img):
        img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img,(3,3),0)  
        edges = cv2.Canny(img, 50, 150, apertureSize = 3)  
        lines = cv2.HoughLines(edges,1,np.pi/180,68) #这里对最后一个参数使用了经验型的值  
        result = img.copy()
        if(lines==None):
                return result
        for line in lines[0]:  
                rho = line[0] #第一个元素是距离rho  
                theta= line[1] #第二个元素是角度theta  
                print rho  
                print theta  
                if  (theta < (np.pi/4. )) or (theta > (3.*np.pi/4.0)): #垂直直线  
                                        #该直线与第一行的交点  
                        pt1 = (int(rho/np.cos(theta)),0)  
                        #该直线与最后一行的焦点  
                        pt2 = (int((rho-result.shape[0]*np.sin(theta))/np.cos(theta)),result.shape[0])  
                        #绘制一条白线  
                        cv2.line( result, pt1, pt2, (255))  
                else: #水平直线  
                        # 该直线与第一列的交点  
                        pt1 = (0,int(rho/np.sin(theta)))  
                        #该直线与最后一列的交点  
                        pt2 = (result.shape[1], int((rho-result.shape[1]*np.cos(theta))/np.sin(theta)))  
                        #绘制一条直线  
                        cv2.line(result, pt1, pt2, (255), 1)
        return result
if __name__ == "__main__":

        # 图像大小
        FRAME_WIDTH = 120
        FRAME_HEIGHT = 90

        # HSV阈值范围
        HUE_BLUE = 240/2
        HUE_YELLOW = 60/2
        HUE_RED = 0
        HUE_RANGE = 15

        SAT_MIN = 100
        SAT_MAX = 255

        VAL_MIN = 50
        VAL_MAX = 255

        # 形态学操作范围
        CLOSE_MASK_SIZE = 30
        KERNEL = np.ones((CLOSE_MASK_SIZE, CLOSE_MASK_SIZE), np.uint8)

        # 初始化相机 图像大小\采集速率
        camera = PiCamera()
        camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=(FRAME_WIDTH, FRAME_HEIGHT))
        time.sleep(0.1)

        # 以BGR格式采集图像
        xx=0
        for rawFrame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
                frame = rawFrame.array
                # 颜色空间变换
                #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # 图像处理
                #processImage(hsv, 1, frame)
                #processImage(hsv, 2, frame)
                #processImage(hsv, 3, frame)

##                cv2.imshow('mask',mask)
                frame=hough(frame)
                cv2.imshow("Frame", frame)
                filepath1=time.strftime( '%Y-%m-%d %X', time.localtime() )
                
                # q键退出
                key = cv2.waitKey(1) & 0xFF
                rawCapture.truncate(0)
                if key == ord("q"):
                        break
                if key == ord('a'):
                        filename=time.strftime( '%Y-%m-%d %X', time.localtime() )   
                        filename+='.jpg'
                        cv2.imwrite(filename, frame)
        cv2.destroyAllWindows()
