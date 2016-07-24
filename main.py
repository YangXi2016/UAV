# -*- coding: utf-8 -*
from define import *
import RPi.GPIO as GPIO
import signal
#from Flight_Control import Serial_Monitor,unlock,lock,baroheight,ultraheight,unheight,send_rcdata,request
from Flight_Control import *
import cv2    
from CameraDetect import *
def INIT():
    global YAW_INIT#,SPEED_X_INIT,SPEED_Y_INIT
    #print data_array
    #print data_array[:]
    #time.sleep(1)
    GPIO.setwarnings(False)
    yaw_sum=0
    for i in range(10):
	yaw_sum+=request_user(2)
	time.sleep(0.05)
    YAW_INIT=yaw_sum/10
    
    '''speed_x_sum=0
    speed_y_sum=0
    time_sum=0
    for i in range (50):
	data=camera_info()
	if data!=0:
	    speed_x_sum+=data[6]
	    speed_y_sum+=data[7]
	    time_sum+=1
	
    SPEED_X_INIT=speed_x_sum/time_sum
    SPEED_Y_INIT=speed_y_sum/time_sum
    
    print 'speed x init:',SPEED_X_INIT
    print 'speed y init:',SPEED_Y_INIT'''
    # BOARD编号方式，基于插座引脚编号  
    GPIO.setmode(GPIO.BOARD)  
    # 输出模式  
    GPIO.setup(PIN_CTR, GPIO.OUT) 
    GPIO.output(PIN_CTR,GPIO.HIGH)
    
    GPIO.setup(PIN_ERR, GPIO.OUT) 
    GPIO.output(PIN_ERR,GPIO.LOW)    

#只是将pid版中参数全部设为0，mode无意义
def Take_off_stable(goal_height,mode=1):
    
    kp=[0,0,0,0,0,0]
    ki=[0,0,0,0,0,0]
    kd=[0,0,0,0,0,0]    

    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    previous_error2=[0,0,0,0]
    error=[0,0,0,0]
    proportion=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    old_derivative=[0,0,0,0]
    
    global position	    #位置外环控制
    global position_times
    global position_i
    position[2]=0
    position[3]=0
    set_x=0
    set_y=0
    output=[0,0,0,0]
    goal=[0,0,0,0]
    get=[0,0,0,0]   
    
    global YAW_INIT

    
    goal[1]=YAW_INIT
    last_time=time.time()
    rc_data[3]+=20
    send_rcdata(rc_data)
    time.sleep(0.1)
    send_rcdata(rc_data)
    time.sleep(0.1)    
    send_rcdata(rc_data)
    time.sleep(0.1)    
    
    ultraheight()
    time.sleep(0.1)
    ultraheight()
    time.sleep(0.1)
    
    unlock()
    time.sleep(0.1)
    unlock()
    #time.sleep(0.1)	
    i=0
    while(1):
	i+=1
	#data=request_user()
	#print data
	#height=data[4]
	[ROL,PIT,YAW,SPEED_Z,height,FLY_MODE,ARMED]=request_user()
	
	get[1]=YAW
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	
	'''去除死区，更改控制曲线
	if(180 > YAW-goal[1]>3):
	    get[1]=YAW-3
	elif(180>360+YAW-goal[1]>3):
	    get[1]=360+YAW-3
	elif(YAW-goal[1]<-3):
	    get[1]=YAW+3
	elif(180<YAW-goal[1]-360<-3):
	    get[1]=YAW+3-360
	else:
	    get[1]=goal[1]
	'''   
	
	#print "height:",height
	if(180>=height>=goal_height):
	    break
	#time.sleep(0.07)
	#data=safe_get(offset_data_queue)
	data=camera_info()
	if(mode==0):
	    if(position_i%position_times==0):
		goal[2]=(set_x-position[2])*kp_x
		goal[3]=(set_y-position[3])*kp_y
		if(goal[2]>SPEED_LIMIT):
		    goal[2]=SPEED_LIMIT
		if(goal[2]<-SPEED_LIMIT):
		    goal[2]=-SPEED_LIMIT
		if(goal[3]>SPEED_LIMIT):
		    goal[3]=SPEED_LIMIT	    
		if(goal[3]<-SPEED_LIMIT):
		    goal[3]=-SPEED_LIMIT		
		position_i=0
		#print "position offset:",position[2],"   ",position[3]
	    else:
		position_i+=1
		

	if(abs(data[6])==127 or abs(data[7]==127)):
	    #get[2]=goal[2]
	    #get[3]=goal[3]
	    pass
	else:
	    get[2]=data[6]
	    get[3]=data[7]
	print "goal speed",goal[2],"   ",goal[3]
	print "speed:",get[2],"   ",get[3]
	    
	dt=time.time()-last_time
	print 'dt:',dt
	last_time=time.time()
	print "yaw","   ",YAW_INIT,'   ',get[1]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    proportion[i]=error[i]-previous_error[i]
	    integral[i]=error[i]*dt
	    derivative[i]=(error[i]-2*previous_error[i]+previous_error2[i])/dt
	    
	    derivative[i]=old_derivative[i]*0.95+derivative[i]*0.05
	    old_derivative[i]=derivative[i]	    
	    
	    output[i]=kp[i]*proportion[i] + ki[i]* integral[i] +kd[i]*derivative[i]
	
	    previous_error2[i]=previous_error[i]
	    previous_error[i]=error[i]
	
	    rc_data[i]+=output[i]
	    #rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    position[i]+=get[i]*dt
	    print(kp[i]* proportion[i],ki[i]* integral[i],kd[i]* derivative[i],position[i])
		
	for i in range (1,4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i]	
		
	'''if(senser_x==0 and senser_y==0):
	    rc_data[2]=OFFSET[2]
	    rc_data[3]=OFFSET[3]'''
	
	rc_data[0]=1600
	send_rcdata(rc_data)
	filehanher.write(str(get[2])+"   "+str(rc_data[2])+"   "+str(get[3])+"   "+str(rc_data[3])+"   "+str(position[2])+"   "+str(position[3])+"   "+str(error[2])+"   "+str(error[3])+"\r\n")
	print "take off"




    '''rc_data[0]=OFFSET[0]
    rc_data[3]=OFFSET[3]+30
    send_rcdata(rc_data)
    while(1):
	data=camera_info()
	break
    time.sleep(3)'''
    

#mode=0 fix_point;mode=1 fix_speed
def Take_off_withpid(goal_height,mode=0):
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    previous_error2=[0,0,0,0]
    error=[0,0,0,0]
    proportion=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    old_derivative=[0,0,0,0]
    
    global position	    #位置外环控制
    global position_times
    global position_i
    position[2]=0
    position[3]=0
    set_x=0
    set_y=0
    output=[0,0,0,0]
    goal=[0,0,0,0]
    get=[0,0,0,0]   
    
    global YAW_INIT

    
    goal[1]=YAW_INIT
    last_time=time.time()
    
    send_rcdata(rc_data)
    time.sleep(0.1)
    send_rcdata(rc_data)
    time.sleep(0.1)    
    send_rcdata(rc_data)
    time.sleep(0.1)    
    
    ultraheight()
    time.sleep(0.1)
    ultraheight()
    time.sleep(0.1)
    
    unlock()
    time.sleep(0.1)
    unlock()
    #time.sleep(0.1)	
    i=0
    while(1):
	i+=1
	#data=request_user()
	#print data
	#height=data[4]
	[ROL,PIT,YAW,SPEED_Z,height,FLY_MODE,ARMED]=request_user()
	
	get[1]=YAW
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	
	'''去除死区，更改控制曲线
	if(180 > YAW-goal[1]>3):
	    get[1]=YAW-3
	elif(180>360+YAW-goal[1]>3):
	    get[1]=360+YAW-3
	elif(YAW-goal[1]<-3):
	    get[1]=YAW+3
	elif(180<YAW-goal[1]-360<-3):
	    get[1]=YAW+3-360
	else:
	    get[1]=goal[1]
	'''   
	
	#print "height:",height
	if(180>=height>=goal_height):
	    break
	#time.sleep(0.07)
	#data=safe_get(offset_data_queue)
	data=camera_info()
	if(mode==0):
	    if(position_i%position_times==0):
		goal[2]=(set_x-position[2])*kp_x
		goal[3]=(set_y-position[3])*kp_y
		if(goal[2]>SPEED_LIMIT):
		    goal[2]=SPEED_LIMIT
		if(goal[2]<-SPEED_LIMIT):
		    goal[2]=-SPEED_LIMIT
		if(goal[3]>SPEED_LIMIT):
		    goal[3]=SPEED_LIMIT	    
		if(goal[3]<-SPEED_LIMIT):
		    goal[3]=-SPEED_LIMIT		
		position_i=0
		#print "position offset:",position[2],"   ",position[3]
	    else:
		position_i+=1
		

	if(abs(data[6])==127 or abs(data[7]==127)):
	    #get[2]=goal[2]
	    #get[3]=goal[3]
	    pass
	else:
	    get[2]=data[6]
	    get[3]=data[7]
	print "goal speed",goal[2],"   ",goal[3]
	print "speed:",get[2],"   ",get[3]
	    
	dt=time.time()-last_time
	print 'dt:',dt
	last_time=time.time()
	print "yaw","   ",YAW_INIT,'   ',get[1]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    proportion[i]=error[i]-previous_error[i]
	    integral[i]=error[i]*dt
	    derivative[i]=(error[i]-2*previous_error[i]+previous_error2[i])/dt
	    
	    derivative[i]=old_derivative[i]*0.95+derivative[i]*0.05
	    old_derivative[i]=derivative[i]	    
	    
	    output[i]=kp[i]*proportion[i] + ki[i]* integral[i] +kd[i]*derivative[i]
	
	    previous_error2[i]=previous_error[i]
	    previous_error[i]=error[i]
	
	    rc_data[i]+=output[i]
	    #rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    position[i]+=get[i]*dt
	    print(kp[i]* proportion[i],ki[i]* integral[i],kd[i]* derivative[i],position[i])
		
	for i in range (1,4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i]	
		
	'''if(senser_x==0 and senser_y==0):
	    rc_data[2]=OFFSET[2]
	    rc_data[3]=OFFSET[3]'''
	
	rc_data[0]=1600
	send_rcdata(rc_data)
	filehanher.write(str(get[2])+"   "+str(rc_data[2])+"   "+str(get[3])+"   "+str(rc_data[3])+"   "+str(position[2])+"   "+str(position[3])+"   "+str(error[2])+"   "+str(error[3])+"\r\n")
	print "take off"




    rc_data[0]=OFFSET[0]
    send_rcdata(rc_data)
def Fix_Point(timeout,signature=-1):
    global offset_array
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    previous_error2=[0,0,0,0]
    error=[0,0,0,0]
    proportion=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    old_derivative=[0,0,0,0]
    
    output=[0,0,0,0]
    goal=[0,0,0,0]
    get=[0,0,0,0]   
    
    #global YAW_INIT
    #YAW_INIT=request_user(0)
    
    goal[1]=YAW_INIT
    last_time=time.time()
    while(1):
	if(time.time()-last_time>timeout):
	    return
	
	[ROL,PIT,YAW,SPEED_Z,height,FLY_MODE,ARMED]=request_user()
	get[1]=YAW
	
	print YAW_INIT,'   ',get[1]
	
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	

	#data=camera_info()
	offset_data=offset_array[:]
	print offset_data
	#data=safe_get(offset_data_queue)
	if(signature==0):
	    if(offset_data[2]>3):
		if(math.sqrt(offset_data[0]*offset_data[0]+offset_data[1]*offset_data[1])<15):
		    return
		else:
		    senser_x=offset_data[0]
		    senser_y=offset_data[1]
	    else:
		senser_x=0
		senser_y=0
	elif(signature==1):
	    if offset_data[5]>7:
		if offset_data[5]>22:
		    thresh=offset_data[5]
		else:
		    thresh=30
		
		if(math.sqrt(offset_data[3]*offset_data[3]+offset_data[4]*offset_data[4])<thresh):
			return
		else:
		    senser_x=offset_data[3]
		    senser_y=offset_data[4]	    
	    else:
		senser_x=0
		senser_y=0	    
	elif(signature==2):
	    if(offset_data[8]>10):
		if(math.sqrt(offset_data[7]*offset_data[7]+offset_data[6]*offset_data[6])<offset_data[8]):
		    return
		else:
		    senser_x=offset_data[6]
		    senser_y=offset_data[7]
	    else:
		senser_x=0
		senser_y=0
		
	else:
	    if(offset_data[5]>8):
		if offset_data[5]>22:
		    thresh=offset_data[5]
		else:
		    thresh=30

		if(math.sqrt(offset_data[3]*offset_data[3]+offset_data[4]*offset_data[4])<thresh):
		    return
		else:
		    senser_x=offset_data[3]
		    senser_y=offset_data[4]
	    elif(offset_data[8]>12):
		if offset_data[8]>30:
		    thresh=offset_data[8]
		else:
		    thresh=30
		if(math.sqrt(offset_data[6]*offset_data[6]+offset_data[7]*offset_data[7])<thresh):
		    return
		else:
		    senser_x=offset_data[6]
		    senser_y=offset_data[7]
	    elif(offset_data[2]>3):
		if(math.sqrt(offset_data[0]*offset_data[0]+offset_data[1]*offset_data[1])<20):
		    return
		else:
		    senser_x=offset_data[0]
		    senser_y=offset_data[1]
	    else:
		senser_x=0
		senser_y=0
	    '''offset=[math.sqrt(offset_data[1]*offset_data[1]+offset_data[0]*offset_data[0]),math.sqrt(offset_data[3]*offset_data[3]+offset_data[4]*offset_data[4]),math.sqrt(offset_data[6]*offset_data[6]+offset_data[7]*offset_data[7])]
	    print offset
	    if(offset[0]<=offset[1] and offset[0]<=offset[2]):
		if(offset[0]<20):
		    return
		else:
		    senser_x=offset_data[0]
		    senser_y=offset_data[1]
	    if(offset[1]<=offset[0] and offset[1]<=offset[2]):
		if(offset[1]<20):
		    return
		else:
		    senser_x=offset_data[3]
		    senser_y=offset_data[4]	    
	    if(offset[2]<=offset[1] and offset[0]>=offset[2]):
		if(offset[2]<20):
		    return
		else:
		    senser_x=offset_data[6]
		    senser_y=offset_data[7]'''
	if(senser_x==-60 and senser_y==-45):
	    #get[2]=goal[2]
	    #get[3]=goal[3]
	    #print "lost object",signature
	    #break
	    pass
	else:	    
	    goal[2]=senser_x*0.09
	    goal[3]=senser_y*0.09
		
	dt=time.time()-last_time
	print "dt:",dt
	last_time=time.time()
	#print "position offset:",goal[2],"   ",goal[3]
	print "position offset",goal[2],'   ',goal[3]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    proportion[i]=error[i]-previous_error[i]
	    integral[i]=error[i]*dt
	    derivative[i]=(error[i]-2*previous_error[i]+previous_error2[i])/dt
	
	    derivative[i]=old_derivative[i]*0.95+derivative[i]*0.05
	    old_derivative[i]=derivative[i]	    
	
	    output[i]=kp[i]*proportion[i] + ki[i]* integral[i] +kd[i]*derivative[i]
	
	    previous_error2[i]=previous_error[i]
	    previous_error[i]=error[i]
	
	    rc_data[i]+=output[i]
	    #rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    position[i]+=get[i]*dt
	    print(kp[i]* proportion[i],ki[i]* integral[i],kd[i]* derivative[i],position[i])

	'''if((radius<min_radius) or (radius>max_radius)):
	    rc_data[2]=OFFSET[2]
	    rc_senser_x=OFFSET[3]'''
		
	for i in range (1,4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i]	
	rc_data[0]=OFFSET[0]
	'''if(senser_x==0 and senser_y==0):
	    rc_data[2]=OFFSET[2]
	    rc_senser_x=OFFSET[3]'''
	send_rcdata(rc_data)	
	print "Fix_point:",signature
	filehanher.write(str(get[2])+"   "+str(rc_data[2])+"   "+str(get[3])+"   "+str(rc_data[3])+"   "+str(goal[2])+"   "+str(goal[3])+"\r\n")
	#filehanher=open(filepath, mode='a')
	#filehanher.write(str(senser_x)+"   "+str(get[2])+"   "+str(rc_data[2])+"   "+str(senser_y)+"   "+str(get[3])+"   "+str(rc_data[3])+"\r\n")
	#filehanher.write(str(YAW_INIT)+"   "+str(YAW)+"   "+str(get[1])+"   "+str(rc_data[1])+"\r\n")
	#filehanher.close()

#signature代表遇到某种颜色跳出
#mode=0代表控位，mode=1代表控速
def Patrol(set_y,timeout,signature,mode=0):
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    previous_error2=[0,0,0,0]
    error=[0,0,0,0]
    proportion=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    old_derivative=[0,0,0,0]
    
    global position	    #位置外环控制
    global position_times
    global position_i
    
    set_x=0
    output=[0,0,0,0]
    goal=[0,0,set_x,set_y]
    get=[0,0,0,0]
    get[1]=YAW_INIT
    begin_time=time.time()
    last_time=time.time()
    while True:
	if time.time()-begin_time>timeout:
	    break
	data=camera_info()
	if(data[signature]==1):
	    return
	elif(data[0]==1 and data[1]==1 and data[2]==1):
	    pass
	elif(data[0]==0 and data[1]==0 and data[2]==0):
	    if mode==0:
		position[2]=0
	elif(data[1]==1):
	    if mode==0:
		position[2]=-70
	    else:
		goal[2]=4
	elif(data[2]==1):
	    if mode==0:
		position[2]=70
	    else:
		goal[2]=-4
	elif(data[0]==1):
	    if mode==0:
		position[2]=0
	    else:
		goal[2]=0	    
	'''print data
	for i in range(6):
	    if(data[i]!=0):
		return'''
	
	'''[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODE,ARMED]=request_user()
	get[1]=YAW
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	'''
	
	if(mode==0):
	    if(position_i%position_times==0):
		goal[2]=position[2]*kp_x
		goal[3]=position[3]*kp_y
		if(goal[2]>SPEED_LIMIT):
		    goal[2]=SPEED_LIMIT
		if(goal[2]<-SPEED_LIMIT):
		    goal[2]=-SPEED_LIMIT
		if(goal[3]>SPEED_LIMIT):
		    goal[3]=SPEED_LIMIT	    
		if(goal[3]<-SPEED_LIMIT):
		    goal[3]=-SPEED_LIMIT		
		position_i=0
		#print "position offset:",position[2],"   ",position[3]
	    else:
		position_i+=1
		
		
	#data=camera_info()
	if(abs(data[6])==127 or abs(data[7]==127)):
	    #get[2]=goal[2]
	    #get[3]=goal[3]
	    pass
	else:
	    get[2]=data[6]
	    get[3]=data[7]
		
	dt=time.time()-last_time
	print 'dt:',dt
	last_time=time.time()
	#print "yaw:",YAW_INIT,'   ',YAW
	print "goal speed:",goal[2],'  ',goal[3]
	print "get speed:",get[2],'   ',get[3]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    proportion[i]=error[i]-previous_error[i]
	    integral[i]=error[i]*dt
	    derivative[i]=(error[i]-2*previous_error[i]+previous_error2[i])/dt
	    
	    derivative[i]=old_derivative[i]*0.9+derivative[i]*0.1
	    old_derivative[i]=derivative[i]	    

	    output[i]=kp[i]*proportion[i] + ki[i]* integral[i] +kd[i]*derivative[i]
	    
	    previous_error2[i]=previous_error[i]
	    previous_error[i]=error[i]
	    
	    rc_data[i]+=output[i]
	    #rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    position[i]+=get[i]*dt
	    print(kp[i]* proportion[i],ki[i]* integral[i],kd[i]* derivative[i],position[i])
	    	
	for i in range (4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i] 
	
	send_rcdata(rc_data)	
	
	print "Patrol"



def PlaneLand():
    rc_data[0]=1450 #本来应该是原来的值，但因为采用GUI事件驱动，可能调用了take_off进程什么的改变了飞机油门，然而由于进程的数据包含，改变后的油门难以得到，折中使用1500数值
    rc_data[1]=1500 #YAW 
    rc_data[2]=1500 #ROL
    rc_data[3]=1500 #PIT
    
    
    send_rcdata(rc_data)
    ##time.sleep(0.1)
    while (rc_data[0]>1100):
	time.sleep(0.2)
	send_rcdata(rc_data)
	rc_data[0]-=100
	
    
    '''
    for i in range(0,5): 
	send_rcdata(rc_data)
	time.sleep(0.1) 
    while (rc_data[0]>1100):
	rc_data[0]-=10
	send_rcdata(rc_data)
	time.sleep(0.05) 
    '''



def PlaneFloat():
    global YAW_INIT,OFFSET
    #保险阈值 
    #max_ang=1.5	    #度
    danger_ang=15   #度
    max_speed=400    #cm/s
    takeoff_speed=500
    max_height=100  #cm
    #takeoff_height=30
    #height_range=10 #cm
    #pid参数

    previous_error=[0,0,0,0]
    error=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    output=[0,0,0,0]
    goal=[0,0,0,0]
    get=[0,0,0,0]
    
    #goal[0] = goal_height# cm  
    goal[1]=YAW_INIT

    last_time=time.time()    
    #图像识别阈值
    min_radius=5
    max_radius=160
    
    
    begin_time=last_time
    while True:
	if time.time()-begin_time>timeout:
	    break
	
	data=request_user()
	#print data
	if data==0 or data[4]>260:
	    time.sleep(0.1)
	    data=request_user()
	[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODE,ARMED]=data

	get[1]=YAW
	
	'''data=safe_get(offset_data_queue)
	if(data==0):
	    get[2]=0
	    get[3]=0
	else:
	    [dx,dy,radius]=data
	    get[2]=getRealLength(dx, ALT_USE)
	    get[3]=getRealLength(dy, ALT_USE)'''
	data=safe_get(offset_data_queue)
	if(data==0):
	    get[2]=goal[2]
	    get[3]=goal[3]
	else:	
	    get[2]=data[0]
	    get[3]=data[1]
	    
	dt=time.time()-last_time
	last_time=time.time()	
	#根据get数据得到pid输出
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    integral[i]+=error[i]*dt
	    derivative[i]=(error[i]-previous_error[i])/dt
	    output[i]=kp[i]* error[i]+ki[i]* integral[i]+kd[i]* derivative[i]
	    #rc_data[i]+=output[i]
	    rc_data[i]=OFFSET[i]+output[i]
	    print(kp[i]* error[i],ki[i]* integral[i],kd[i]* derivative[i])
	    
	'''if((radius<min_radius) or (radius>max_radius)):
	    rc_data[2]=OFFSET[2]
	    rc_data[3]=OFFSET[3]'''
		
	for i in range (4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i] 
	
	send_rcdata(rc_data)	
	
	print "Float"




def test():
    delt=25
    send_rcdata(rc_data)
    height_mode=1
    while True:
        cmd=raw_input("add(a) or decrease:")
	if(cmd==''):
	    lock()
	    print('飞机已上锁')
	    #out_queue.put('close serial')
	    #safe_put(out_queue, 'close serial')  	    

	    Serial_process.terminate()
	    print("已退出")
	    break
        elif(cmd[0]=='u'):
	    print time.time()
            unlock()
	    print time.time()
        elif(cmd=='h'):
            baroheight()
        elif(cmd=='hh'):
            ultraheight()
        elif(cmd=='hhh'):
            unheight()
        elif(cmd[0]=='a'):
            if(len(cmd)>1):
                temp_delt=int(cmd[1:])
                rc_data[0]+=temp_delt
            else:
                rc_data[0]+=delt
            send_rcdata(rc_data)

        elif(cmd[0]=='d'):
            if(len(cmd)>1):
                temp_delt=int(cmd[1:])
                rc_data[0]-=temp_delt
            else:
                rc_data[0]-=delt
            send_rcdata(rc_data)

        elif(cmd[0]=='q'):
            if(len(cmd)>1):
                temp_delt=int(cmd[1:])
                rc_data[1]+=temp_delt
            else:
                rc_data[1]+=1
            send_rcdata(rc_data)
            
        elif(cmd[0]==' '):
            rc_data[0]=1500
            send_rcdata(rc_data)
        elif(cmd[0]=='r'):
	    
            request_type=ord(cmd[1])-ord('0')
	    data=request(request_type)
	    print data
        else:
            lock()
	status=request_user()
	print status



def myPlaneFloat(timeout):
    last_time=time.time()
    while(time.time()-last_time>timeout):
	for i in range(4):
	    rc_data[i]=OFFSET[i]
	send_rcdata(rc_data)
	time.sleep(0.4)
    

def Fly():
    INIT()
    Take_off_withpid(100,mode=0)
    #Take_off_stable(95)
    SetFly(0, -13, 2111,mode=1)
    #Patrol(-70, 1111, 3,mode=0)
    #Fix_Point(1111,signature=0)
    #GPIO.output(PIN_CTR,GPIO.LOW)
    '''rc_data[3]+=30
    send_rcdata(rc_data)
    time.sleep(0.4)
    rc_data[3]-=35
    send_rcdata(rc_data)
    while(1):
	Patrol(-6,1250,signature=3,mode=1)
	#rc_data[0:4]=OFFSET[0:4]
	#rc_data[3]-=45
	#send_rcdata(rc_data)
	while(1):
	    offset_data=offset_array[:]
	    if(offset_data[5]>7 or offset_data[8]>12 or offset_data[2]>3):
		break
	Fix_Point(30)
	#time.sleep(0.5)
	rc_data[3]+=20
	send_rcdata(rc_data)
	time.sleep(0.5)
	GPIO.output(PIN_CTR,GPIO.LOW)
	rc_data[3]-=20
	send_rcdata(rc_data)
	Patrol(8, 1250,signature=4,mode=1)
	GPIO.output(PIN_CTR,GPIO.HIGH)
	for i in range(10):
	    send_rcdata(rc_data)
	    time.sleep(0.4)
	rc_data[0:4]=OFFSET[0:4]
	send_rcdata(rc_data)
	time.sleep(3)'''
    '''for i in range(3):
	GPIO.output(PIN_ERR,GPIO.HIGH)
	time.sleep(0.2)
	GPIO.output(PIN_ERR,GPIO.LOW)
	time.sleep(0.2)
    FlyToGoalArea(4,0,10)
    FlyToGoalArea(-8,0,10)
    for i in range(3):
	GPIO.output(PIN_ERR,GPIO.HIGH)
	time.sleep(0.2)
	GPIO.output(PIN_ERR,GPIO.LOW)
	time.sleep(0.2)
    FlyToGoalArea(0,0,10)'''
    #Fix_Point(0)
    '''rc_data[0:4]=OFFSET[0:4]
    rc_data[3]=OFFSET[3]+35
    FlyToGoalArea(30,0, 10)
    
    rc_data[0:4]=OFFSET[0:4]
    rc_data[3]=OFFSET[3]-35
    FlyToGoalArea(-30,0, 2)
    rc_data[0:4]=OFFSET[0:4]
    send_rcdata(rc_data)
    time.sleep(0.1)
    send_rcdata(rc_data)
    while(1):
	data=camera_info()
	if(data[0]!=0 and data[1]!=0):
	    GPIO.output(PIN_CTR,GPIO.LOW)
	    Fix_Point(0)
	elif(data[2]!=0 and data[3]!=0):
	    GPIO.output(PIN_CTR,GPIO.LOW)
	    Fix_Point(1)
	elif(data[4]!=0 and data[5]!=0):
	    GPIO.output(PIN_CTR,GPIO.LOW)
	    Fix_Point(2)
	else:
	    GPIO.output(PIN_CTR,GPIO.HIGH)'''
    #Fix_Point()

#mode=0代表控位 mode=1代表控速
def SetFly(set_x,set_y,timeout,mode=0):
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    previous_error2=[0,0,0,0]
    error=[0,0,0,0]
    proportion=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    old_derivative=[0,0,0,0]

    global position	    #位置外环控制
    global position_times
    global position_i

    output=[0,0,0,0]
    goal=[0,YAW_INIT,set_x,set_y]
    get=[0,0,0,0]
    get[1]=YAW_INIT
    begin_time=time.time()
    last_time=time.time()
    while True:
	if time.time()-begin_time>timeout:
	    break
	data=camera_info()
	'''print data
	for i in range(3):
	    if(data[i]!=0):
		pass
		#return'''
	
	'''[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODE,ARMED]=request_user()
	get[1]=YAW
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	'''
	    
	#data=camera_info()
	if(abs(data[6])==127 or abs(data[7]==127)):
	    #get[2]=goal[2]
	    #get[3]=goal[3]
	    pass
	else:
	    get[2]=data[6]
	    get[3]=data[7]
	
	if(mode==0):
	    if(position_i%position_times==0):
		goal[2]=(set_x-position[2])*kp_x
		goal[3]=(set_y-position[3])*kp_y
		if(goal[2]>SPEED_LIMIT):
		    goal[2]=SPEED_LIMIT
		if(goal[2]<-SPEED_LIMIT):
		    goal[2]=-SPEED_LIMIT
		if(goal[3]>SPEED_LIMIT):
		    goal[3]=SPEED_LIMIT	    
		if(goal[3]<-SPEED_LIMIT):
		    goal[3]=-SPEED_LIMIT		
		position_i=0
		#print "position offset:",position[2],"   ",position[3]
	    else:
		position_i+=1	    
	    
	dt=time.time()-last_time
	print 'dt:',dt
	last_time=time.time()
	#print "yaw:",YAW_INIT,'   ',YAW
	print "goal speed",goal[2],'   ',goal[3]
	print "speed:",get[2],'   ',get[3]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    proportion[i]=error[i]-previous_error[i]
	    integral[i]=error[i]*dt
	    derivative[i]=(error[i]-2*previous_error[i]+previous_error2[i])/dt
	    
	    derivative[i]=old_derivative[i]*0.9+derivative[i]*0.1
	    old_derivative[i]=derivative[i]	    

	    output[i]=kp[i]*proportion[i] + ki[i]* integral[i] +kd[i]*derivative[i]
	    
	    previous_error2[i]=previous_error[i]
	    previous_error[i]=error[i]
	    
	    rc_data[i]+=output[i]
	    #rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    position[i]+=get[i]*dt
	    print(kp[i]* proportion[i],ki[i]* integral[i],kd[i]* derivative[i],position[i])
	    
	for i in range (1,4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i]		
	send_rcdata(rc_data)	
	#time.sleep(0.1)
	filehanher.write(str(get[2])+"   "+str(rc_data[2])+"   "+str(get[3])+"   "+str(rc_data[3])+"   "+str(position[2])+"   "+str(position[3])+"   "+str(error[2])+"   "+str(error[3])+"\r\n")
	print "KeepFly"

  
    
def Key_function(event):
    key=event.char
    print(key)
    print(ord(key))
    try:
	#print(event.time,ord(key))
	listbox.insert(END, key)
	if(key=='t'):
	    #print("t")
	    Fly_process.start()
	else:
	    try:
		#Camera_process.terminate()
		Fly_process.terminate()

		
	    except Exception, exc:
		print Exception, ":", exc
		print("terminate error")
		
	    if(ord(key)==13):
		PlaneLand()
		time.sleep(0.05)
		lock()
		time.sleep(0.05)
		lock()	    
	    lock()
	    time.sleep(0.05)
	    lock()
	    time.sleep(0.05)	    
	    lock()
	    time.sleep(0.05)	
	
	    try:
		Serial_process.terminate()
		#Detect_process.terminate()
	    except Exception, exc:
		print Exception, ":", exc
	    GPIO.output(PIN_CTR,GPIO.LOW)
	    master.destroy()
	    
    except:
	pass

def sigint_handler(signum, frame):
    global is_sigint_up
    is_sigint_up = True
    print 'catched interrupt signal!'

signal.signal(signal.SIGINT, sigint_handler)
is_sigint_up = False

def test_senser():
    last_time=time.time()
    while(1):
	'''data=offset_array[:]
	offset_array[0]=255
	if(data[0]!=255):
	    dt=time.time()-last_time
	    print dt
	    print data
	    last_time=time.time()'''
	data=camera_info()
	dt=time.time()-last_time
	last_time=time.time()
	print dt
	print data	

def self_check():
    global offset_array
    while(1):
	data=camera_info()
	print data
	data=request_user()
	print data
	adata=offset_array[:]
	print adata

if __name__ == '__main__':
    global senser_array,data_array,out_array,offset_array
    print time.strftime( '%Y-%m-%d %X', time.localtime() )
    mp.freeze_support()
    print("Create new process as reader!")                   
    Serial_process = mp.Process(target=Serial_Monitor, args=(senser_array,data_array,out_array))
    Serial_process.start()
    Camera_process = mp.Process(target=Offset_Detect,args=(offset_array,))
    Camera_process.start()
    #print data_array
    #print data_array[:]
    #print out_array
    #print out_array[:]
    INIT()
    Fly_process = mp.Process(target=Fly, args=())
    #Detect_process = mp.Process(target=Offset_Detect, args=(offset_data_queue,))
    #sDetect_process.start()
    
    #test()
    cmd=raw_input("enter 't' to take off:")
    if cmd=='t': 
	Fly_process.start()
        
    '''times=200
    begin_time=time.time()
    while(times>0):
	data=camera_info()
	print data
	times-=1
    print (time.time()-begin_time)'''
    
    while(1):
	try:
	    if is_sigint_up==True:
		print 'My KeyboardInterrupt'
		try:
		    #Camera_process.terminate()
		    Fly_process.terminate()
		
		except Exception, exc:
		    print Exception, ":", exc
		    print("terminate error")
		PlaneLand()
		lock()
		time.sleep(0.05)
		lock()	    
		lock()
		time.sleep(0.05)
		lock()
		time.sleep(0.05)	    
		lock()
		time.sleep(0.05)	
		
		try:
		    Serial_process.terminate()
		    #Detect_process.terminate()
		except Exception, exc:
		    print "err1:",Exception, ":", exc
		    
		try:
		    Camera_process.terminate()
		    #Detect_process.terminate()
		except Exception, exc:
		    print "err1:",Exception, ":", exc	
		    
		GPIO.output(PIN_CTR,GPIO.LOW)
		time.sleep(2)
		GPIO.output(PIN_ERR,GPIO.LOW)
		break
	except Exception, exc:
	    print "err2:",Exception, ":", exc

    '''data=[0,0,0,0,0]
    old_data=camera_info()
    filepath1=time.strftime( '%Y-%m-%d %X', time.localtime() )
    filepath2=filepath1
    filepath1+='sourcedata.txt'
    filepath2+='resultdata.txt'

    while(1):
	file1=open(filepath1,'a')
	file2=open(filepath2,'a')	
	new_data=camera_info()
	for i in range(5):
	    data[i]=int(0.9*old_data[i]+0.1*new_data[i])
	    file1.write(str(new_data[i])+'  ')
	    file2.write(str(data[i])+'  ')
	file1.write('\r\n')
	file2.write('\r\n')
	file1.close()
	file2.close()
	print data[0],' ',data[3],' ',data[4]
	old_data=data'''
    
    #master.bind("<Key>", Key_function)
    #mainloop()

