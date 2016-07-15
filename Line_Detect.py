#coding=utf-8
import cv2
import numpy as np  

#左右偏移度，是否检测到拐角

def test():
    output=[0,0]
    img = cv2.imread("test4.jpg",0)
    gray = cv2.GaussianBlur(img,(3,3),0)
    
    ret,thresh=cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,9,-10)    
    
    #cv2.imshow('threshold', thresh)
    #cv2.waitKey(0)
    
    edges = cv2.Canny(thresh, 40, 150, apertureSize = 3)
    #cv2.imshow('edges', edges)
    #cv2.waitKey(0)
    '''minLineLength = 30
    maxLineGap = 10
    lines = cv2.HoughLinesP(edges,1,np.pi/180,15,minLineLength,maxLineGap)'''
    
    lines = cv2.HoughLines(edges,1,np.pi/180,30) #这里对最后一个参数使用了经验型的值
    
    if lines==None:
        return [0,0]
    result = img.copy()
    #result = np.zeros(img.shape, np.uint8)
    theta1=[]
    theta2=[]
    position_x=[]
    for line in lines[0]:  
        rho = line[0] #第一个元素是距离rho  
        theta= line[1] #第二个元素是角度theta
        #print rho  
        #print theta
        if  (theta < (np.pi/4. )) or (theta > (3.*np.pi/4.0)): #垂直直线  
            '''        #该直线与第一行的交点  
            pt1 = (int(rho/np.cos(theta)),0)  
            #该直线与最后一行的焦点  
            pt2 = (int((rho-result.shape[0]*np.sin(theta))/np.cos(theta)),result.shape[0])  
            #绘制一条白线  
            cv2.line( result, pt1, pt2, (255)) ''' 
            theta1.append(theta)
            position_x.append(int((2*rho-result.shape[0]*np.sin(theta))/2/np.cos(theta)*255/240))
        else: #水平直线  
            '''# 该直线与第一列的交点  
            pt1 = (0,int(rho/np.sin(theta)))  
            #该直线与最后一列的交点  
            pt2 = (result.shape[1], int((rho-result.shape[1]*np.cos(theta))/np.sin(theta)))  
            #绘制一条直线  
            cv2.line(result, pt1, pt2, (255), 1)  '''
            theta2.append(theta)
    #print theta1
    #print theta2
    if(len(position_x)==0):
        output=[0,0]
        return output
    
    output[0]=sum(position_x)/len(position_x)
    
    if((len(theta1)!=0) and (len(theta2) !=0)):
        if 1.4<abs(sum(theta1)/len(theta1)-sum(theta2)/len(theta2))<1.75:
            #print 'Yes'
            output[1]=1
               
    
    
    cv2.circle(result,(output[0],90),2,255,3)
    cv2.imshow('Result', result)
    cv2.waitKey(0)
 
    cv2.destroyAllWindows()
    return output 

if __name__=='__main__':
    output=test()
    print output