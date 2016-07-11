# -*- coding: utf-8 -*
from define import *
import RPi.GPIO as GPIO
#from Flight_Control import Serial_Monitor,unlock,lock,baroheight,ultraheight,unheight,send_rcdata,request
from Flight_Control import *
import cv2    
from CameraDetect import *
def INIT():
    global YAW_INIT
    yaw_sum=0
    for i in range(5):
	yaw_sum+=request_user(2)
    YAW_INIT=yaw_sum/5
    
    # BOARD编号方式，基于插座引脚编号  
    GPIO.setmode(GPIO.BOARD)  
    # 输出模式  
    GPIO.setup(PIN_CTR, GPIO.OUT) 
    GPIO.output(PIN_CTR,GPIO.HIGH)
def Take_off_stable():
    global YAW_INIT
    YAW_INIT=request_user(0)
    
    #max_ang=1.5	    #度
    danger_ang=25    #度
    max_speed=500    #cm/s
    takeoff_speed=500
    max_height=150  #cm
    takeoff_height=75
    
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
	data=request_user()
	#print data
	height=data[4]
	print "height:",height
	if(height>45 or i>55):
	    break
	rc_data[0]=1660
	send_rcdata(rc_data)
	time.sleep(0.1)
	i+=1
	
    '''rc_data[0]=1500
    for i in range(300):
	send_rcdata(rc_data)
	time.sleep(0.1)
'''
#model=0 fix_point;model=1 fix_speed
def Take_off_withpid(goal_height,model=0):
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    error=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
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
	[ROL,PIT,YAW,SPEED_Z,height,FLY_MODEL,ARMED]=request_user()
	
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
	
	print "height:",height
	if(height>=goal_height or i>45):
	    break
	
	#data=safe_get(offset_data_queue)
	data=camera_info()
	if(model==0):
	    senser_x=data[0]
	    senser_y=data[1]
	    if(senser_x==0 and senser_y==0):
		get[2]=goal[2]
		get[3]=goal[3]
	    else:
		get[2]=senser_x
		get[3]=senser_y
	elif(model==1):
	    get[2]=data[6]
	    get[3]=data[7]
	    
	dt=time.time()-last_time
	last_time=time.time()
	print YAW_INIT,'   ',get[1]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    integral[i]+=error[i]*dt
	    derivative[i]=(error[i]-previous_error[i])/dt
	    output[i]=kp[i]* error[i]+ki[i]* integral[i]+kd[i]* derivative[i]
	    rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]+=output[i]
	    #rc_data[i]=OFFSET[i]+output[i]
	    print(kp[i]* error[i],ki[i]* integral[i],kd[i]* derivative[i])
	    
	'''if((radius<min_radius) or (radius>max_radius)):
	    rc_data[2]=OFFSET[2]
	    rc_data[3]=OFFSET[3]'''
		
	for i in range (1,4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i]	
		
	'''if(senser_x==0 and senser_y==0):
	    rc_data[2]=OFFSET[2]
	    rc_data[3]=OFFSET[3]'''
	rc_data[0]=1660
	
	send_rcdata(rc_data)

	

def Fix_Point(signature):
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    error=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    output=[0,0,0,0]
    goal=[0,0,0,0]
    get=[0,0,0,0]   
    
    #global YAW_INIT
    #YAW_INIT=request_user(0)
    
    goal[1]=YAW_INIT
    last_time=time.time()
    #filepath=time.strftime( '%Y-%m-%d %X', time.localtime())
    #filehanher=open(filepath, mode='a')
    while(1):
	[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODEL,ARMED]=request_user()
	get[1]=YAW
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	

	data=camera_info()
	#data=safe_get(offset_data_queue)
	if(signature==0):
	    senser_x=data[0]
	    senser_y=data[1]
	elif(signature==1):
	    senser_x=data[2]
	    senser_y=data[3]	    
	else:
	    senser_x=data[4]
	    senser_y=data[5]
	    
	if(senser_x==0 and senser_y==0):
	    #get[2]=goal[2]
	    #get[3]=goal[3]
	    print "lost object",signature
	    break
	else:
	    get[2]=senser_x*kp_x+data[6]
	    get[3]=senser_y*kp_y+data[7]
		
	dt=time.time()-last_time
	last_time=time.time()
	print "position factor",senser_x*kp_x,'   ',senser_y*kp_y
	print "speed",data[6],'   ',data[7]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    integral[i]+=error[i]*dt
	    derivative[i]=(error[i]-previous_error[i])/dt
	    output[i]=kp[i]* error[i]+ki[i]* integral[i]+kd[i]* derivative[i]
	    rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    print(kp[i]* error[i],ki[i]* integral[i],kd[i]* derivative[i])
	    
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
	
	#filehanher=open(filepath, mode='a')
	#filehanher.write(str(senser_x)+"   "+str(get[2])+"   "+str(rc_data[2])+"   "+str(senser_y)+"   "+str(get[3])+"   "+str(rc_data[3])+"\r\n")
	#filehanher.write(str(YAW_INIT)+"   "+str(YAW)+"   "+str(get[1])+"   "+str(rc_data[1])+"\r\n")
	#filehanher.close()

def Patrol(speed_y):
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
    
    begin_time=last_time
    while True:
	if time.time()-begin_time>timeout:
	    break
	data=request_user()
	#print data
	if data==0 or data[4]>260:
	    time.sleep(0.1)
	    data=request_user()
	[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODEL,ARMED]=data

	get[1]=YAW
	
	data=camera_info()
	if(data[3]==0 or data[4]==0):
	    get[2]=goal[2]
	    get[3]=goal[3]
	else:
	    get[2]=data[3]
	    get[3]=data[4]
	    
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
	
	print "Patrol"



def PlaneLand():
    rc_data[0]=1450 #本来应该是原来的值，但因为采用GUI事件驱动，可能调用了take_off进程什么的改变了飞机油门，然而由于进程的数据包含，改变后的油门难以得到，折中使用1500数值
    rc_data[1]=1500 #YAW 
    rc_data[2]=1500 #ROL
    rc_data[3]=1500 #PIT
    
    
    send_rcdata(rc_data)
    time.sleep(0.2)
    while (rc_data[0]>1100):
	
	send_rcdata(rc_data)
	rc_data[0]-=200
	time.sleep(0.8)
    
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
	[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODEL,ARMED]=data

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
	    safe_put(out_queue, 'close serial')  	    

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
    Take_off_withpid(65)
    #Fix_Point()
    rc_data[0:4]=OFFSET[0:4]
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
	    GPIO.output(PIN_CTR,GPIO.HIGH)
    #Fix_Point()
def FlyToGoalArea(speed_x,speed_y,timeout):
    #thr,yaw,tol,pit
    previous_error=[0,0,0,0]
    error=[0,0,0,0]
    integral=[0,0,0,0]
    derivative=[0,0,0,0]
    output=[0,0,0,0]
    goal=[YAW_INIT,0,speed_x,speed_y]
    get=[0,0,0,0]
    
    begin_time=time.time()
    last_time=time.time()
    while True:
	if time.time()-begin_time>timeout:
	    break
	data=camera_info()
	print data
	for i in range(6):
	    if(data[i]!=0):
		return
	
	[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODEL,ARMED]=request_user()
	get[1]=YAW
	if(YAW-goal[1]>180):
	    get[1]-=360
	if(YAW-goal[1]<-180):
	    get[1]+=360	
	    
	data=camera_info()
	get[2]=data[6]
	get[3]=data[7]
		
	dt=time.time()-last_time
	last_time=time.time()
	print "yaw:",YAW_INIT,'   ',YAW
	print "speed:",get[2],'   ',get[3]
	for i in range(1,4):
	    #print(get[i],goal[i])
	    error[i]=goal[i]-get[i]
	    integral[i]+=error[i]*dt
	    derivative[i]=(error[i]-previous_error[i])/dt
	    output[i]=kp[i]* error[i]+ki[i]* integral[i]+kd[i]* derivative[i]
	    #rc_data[i]+=output[i]
	    rc_data[i]+=output[i]*(0.55+0.45*output[i]/RANGE[i])
	    #rc_data[i]=OFFSET[i]+output[i]
	    print(kp[i]* error[i],ki[i]* integral[i],kd[i]* derivative[i])
	    
	for i in range (1,4):
	    if rc_data[i]>OFFSET[i]+RANGE[i]:
		rc_data[i] = OFFSET[i]+RANGE[i]
	    if rc_data[i]<OFFSET[i]-RANGE[i]:
		rc_data[i] = OFFSET[i]-RANGE[i]		
	send_rcdata(rc_data)	
	time.sleep(0.1)
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

if __name__ == '__main__':
    print time.strftime( '%Y-%m-%d %X', time.localtime() )
    mp.freeze_support()
    print("Create new process as reader!")                   
    Serial_process = mp.Process(target=Serial_Monitor, args=(senser_queue,data_queue,out_queue))
    Serial_process.start() 
    INIT()
    Fly_process = mp.Process(target=Fly, args=())
    #Detect_process = mp.Process(target=Offset_Detect, args=(offset_data_queue,))
    #Detect_process.start()
    
    while(1):
	data=camera_info()
	print data
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
    
    master.bind("<Key>", Key_function)
    mainloop()

