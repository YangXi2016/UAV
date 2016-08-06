# -*- coding: utf-8 -*
from define import *
import time
from Flight_Serial import Serial_Monitor,assembly_cmd 
import numpy as np

# Read msg from the queue and print it out
def Wait_Check():
    return 1


def Wait_Data(data_array,data_type):
    #global data_array
    last_time=time.time()
    while(time.time()-last_time<1):
        #if(data_queue.empty()==False):       
        frame=data_array[:]
        #print frame
        if(frame==0):
            continue
        elif(frame[2]==data_type):
            return frame[4:-1]
    return 0 




def unlock():
    global out_array
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_unlock)
        #out_queue.put(cmd)
        #safe_put(out_queue, cmd)
        out_array[:len(cmd)]=map(ord,cmd)
        #print cmd
        state =1# request_user(6)
        #print state
        if (state ==1):
            break
    #return state

    
def lock():
    global out_array
    print("上锁")
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_lock)
        #out_queue.put(cmd)
        out_array[:len(cmd)]=map(ord,cmd)
        state =1# request_user(6)
        if state ==1:
            print state
            return state

def baroheight():
    global out_array
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_baroheight_mode)
        #out_queue.put(cmd)
        out_array[:len(cmd)]=map(ord,cmd)
        #state = Wait_Check(data_queue, COMMAND, Sum)
        state=1
        if state ==1:
            height_mode=1
            print state
            return state

def ultraheight():
    global out_array
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_ultraheight_mode)
        #out_queue.put(cmd)
        out_array[:len(cmd)]=map(ord,cmd)
        state =1# Wait_Check(data_queue, COMMAND, Sum)
        if state ==1:
            height_mode=2
            print state
            return state

def unheight():
    global out_array
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_unheight_mode)
        #out_queue.put(cmd)
        out_array[:len(cmd)]=map(ord,cmd)
        state = Wait_Check(data_queue, COMMAND, Sum)
        if state ==1:
            height_mode=0
            print state
            return state
def send_rcdata(rc_data):
    #global out_array
    #print time.clock()
    rc_data=map(int,rc_data)
    print rc_data[0:4]
    #print out_array
    #print out_array[:]
    #listbox.insert(0, "rc_data")
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, RCDATA, rc_data)
        #out_queue.put(cmd)
        #print cmd
        out_array[:len(cmd)]=map(ord,cmd)
        '''print out_array
        for i in range(len(cmd)):
            print out_array[i]'''        
        #print out_array[:len(cmd)]
        state =1# Wait_Check(data_queue, RCDATA, Sum)
        if state ==1:
            print state
            return state
        

def request_user(id=None):
    global data_array
    data=data_array[:]
    #data=request(1)
    #print data
    if data==0:
        return 0
    ROL=np.int16(((data[0]<<8)+data[1]))/100.0
    PIT=np.int16(((data[2]<<8)+data[3]))/100.0
    YAW=np.int16(((data[4]<<8)+data[5]))/100.0
    SPEED_Z=np.int16(((data[6]<<8)+data[7]))
    ALT_USE=((data[8]<<24)+(data[9]<<16)+(data[10]<<8)+data[11])
    print "height",ALT_USE
    FLY_MODEL=data[12]
    ARMED=data[13]
    result=[ROL,PIT,YAW,SPEED_Z,ALT_USE,FLY_MODEL,ARMED]
    print result
    if(id==None):
        return result
    else:
        return result[id]
