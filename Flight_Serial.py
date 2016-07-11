    # -*- coding: utf-8 -*
import serial
import time
from define import *

def assembly_cmd(head_send,cmd_type,cmd_data):
    cmd=[]
    data_len=0
    for i in head_send:
	cmd.append(chr(i))
    cmd.append(chr(cmd_type))
    for j in cmd_data:
	if(cmd_type==RCDATA):
	    cmd.append(chr(j>>8));
	    cmd.append(chr(j&255));
	    data_len+=2
	else:
	    cmd.append(chr(j))
	    data_len+=1
    cmd.insert(3,chr(data_len))
    Sum=0
    for j in cmd:
	Sum+=ord(j)
	Sum&=255
    cmd.append(chr(Sum))
    return cmd,Sum

def Send_cmd(ser,cmd):
    ser.flushInput()
    for cmdi in cmd:
	ser.write(cmdi)



#Write msg with number into the queue
def Serial_Monitor(senser_queue,data_queue,out_queue):
#def writer(senser_queue,data_queue):
    ser=serial.Serial(SER_COM, 115200,timeout=0.5)
    state=0
    last_time=time.time()
    frame=[]
    data_cnt=0
    while(1):
	if(out_queue.empty()==False):
	    #cmd=safe_get(out_queue)#out_queue.get()
	    try:        
		cmd=out_queue.get(False) 
		#print(cmd)
	    except:         
		pass            
	    Send_cmd(ser,cmd)        
	if(ser.inWaiting()>0):
	    data=ser.read()
	    if(state==0 and ord(data)==HEAD_RECE[0]):
		state=1
		frame.append(ord(data))
	    elif(state==1 and ord(data)==HEAD_RECE[1]):
		state=2
		frame.append(ord(data))
	    elif(state==2 and ord(data)<=TYPE_END):
		state=3
		frame.append(ord(data))
	    elif(state==3 and ord(data)<=255):
		state=4
		frame.append(ord(data))
		data_len=ord(data)
	    elif(state==4 and data_len>0):
		data_len-=1
		frame.append(ord(data))
		data_cnt+=1
		if(data_len==0):
		    state=5    
	    elif(state==5):
		state = 0
		frame.append(ord(data))
		#print(map(hex,frame))
		#if(frame[2]==0x02):
		Sum=0
		for data in frame[:-1]:
		    Sum+=data
		if((Sum&0xFF)!=frame[-1]):
		    pass
		elif(frame[2]==0xF1):
		    #print frame
		    try:
			if(senser_queue.empty()==False):
			    senser_queue.get(False)	
			senser_queue.put(frame[4:16],False)
		    except:         
			    print("senser_queue.put error")
			    
		    try:
			if(data_queue.empty()==False):
			    data_queue.get(False)
			data_queue.put(frame[10:-1],False)
			
		    except:
			print("data_queue.put error")
                 
		else:
		    pass  
		frame=[]
		ser.flushInput()
	    else:
		state=0
