from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv,cv2
import numpy as np
import serial
import math
import copy


#摄像头向后运动，y值为正；摄像头向右运动，x值为正。

FRAME_WIDTH = 160 #default resolution is 1920*1080 1280*720 640*480
FRAME_HEIGHT = 120

if __name__ == "__main__":

    # initate camera
    camera = PiCamera()
    camera.resolution = (FRAME_WIDTH, FRAME_HEIGHT)
    camera.framerate = 2
    camera.awb_mode = "incandescent"
    rawCapture = PiRGBArray(camera, size=(FRAME_WIDTH, FRAME_HEIGHT))
    time.sleep(0.1)

    #first frame
    #camera.capture(rawCapture, format="bgr")
    #preColor = rawCapture.array
    #cv2.imshow("test",preColor)
    #cv2.waitKey(0)
    #camera.close()
    preColor = None
    last_time=time.time()
    for rawFrame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        nextColor = rawFrame.array
        
        #cv2.waitKey(0)
        if preColor is None:
            preColor = copy.deepcopy(nextColor)
            rawCapture.truncate(0)# inevitable
            print 1
            continue
            
        preGray = cv2.cvtColor(preColor,cv2.COLOR_BGR2GRAY)
        nextGray = cv2.cvtColor(nextColor,cv2.COLOR_BGR2GRAY)

        prePts = cv2.goodFeaturesToTrack(preGray,100,0.01,10)
        if prePts is None:
            rawCapture.truncate(0)# inevitable
            print 2
            continue
        
        #print prePts.shape,len(prePts)
        prePts = np.float32(np.around(prePts))
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.03) 
        cv2.cornerSubPix(preGray,prePts,(10,10),(-1,-1),criteria)
        nextPts, status, err = cv2.calcOpticalFlowPyrLK(preGray,nextGray,prePts)
        
        prePts = np.int32(np.around(prePts))
        nextPts = np.int32(np.around(nextPts))
        #print nextPts.shape,len(nextPts),nextPts
        print "x y:"
        sum_speed_x=0
        sum_speed_y=0
        sum_count=0
        for i in range(len(status)):
            if status[i]!=0:
                derection_x = prePts[i,0,0] - nextPts[i,0,0]
                derection_y = prePts[i,0,1] - nextPts[i,0,1]
                sum_speed_x+=derection_x
                sum_speed_y+=derection_y
                sum_count+=1
                #print prePts[i,0,0],prePts[i,0,0]
                #cv2.circle(nextColor,(i[0],i[1]),i[2],(0,0,255),2)
                cv2.line(nextColor,(prePts[i,0,0],prePts[i,0,1]),(nextPts[i,0,0],nextPts[i,0,1]),(0,255,0))
        if sum_count==0:
            speed_x=0
            speed_y=0
        else:
            speed_x=sum_speed_x*1.0/sum_count
            speed_y=sum_speed_y*1.0/sum_count
        print speed_x,speed_y
        cv2.line(nextColor,(FRAME_WIDTH/2,FRAME_HEIGHT/2),(FRAME_WIDTH/2+int(speed_x*100),int(FRAME_HEIGHT/2+speed_y*100)),(255,0,0))
        cv2.imshow("nextColor",nextColor)
        cv2.imshow("nextGray",nextGray)
        preColor = copy.deepcopy(nextColor)
        
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)# inevitable
        if key == ord("q"):
            break
    camera.close()
    cv2.destroyAllWindows()
