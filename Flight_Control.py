# -*- coding: utf-8 -*
from define import *
import time
from Flight_Serial import Serial_Monitor,assembly_cmd 
import numpy as np

# Read msg from the queue and print it out
def Wait_Check(data_queue,fream_head,check_sum):
    return 1


def Wait_Data(data_queue,data_type):
    last_time=time.time()
    while(time.time()-last_time<1):
        #if(data_queue.empty()==False):       
        frame=safe_get(data_queue)
        #print frame
        if(frame==0):
            continue
        elif(frame[2]==data_type):
            return frame[4:-1]
    return 0 




def unlock():
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_unlock)
        #out_queue.put(cmd)
        safe_put(out_queue, cmd)
        #print cmd
        state =1# request_user(6)
        #print state
        if (state ==1):
            break
    #return state

    
def lock():
    print("上锁")
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_lock)
        #out_queue.put(cmd)
        safe_put(out_queue,cmd)
        state =1# request_user(6)
        if state ==1:
            print state
            return state

def baroheight():
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_baroheight_mode)
        #out_queue.put(cmd)
        safe_put(out_queue,cmd)
        state = Wait_Check(data_queue, COMMAND, Sum)
        if state ==1:
            height_mode=1
            print state
            return state

def ultraheight():
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_ultraheight_mode)
        #out_queue.put(cmd)
        safe_put(out_queue,cmd)
        state =1# Wait_Check(data_queue, COMMAND, Sum)
        if state ==1:
            height_mode=2
            print state
            return state

def unheight():
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, COMMAND, cmd_unheight_mode)
        #out_queue.put(cmd)
        safe_put(out_queue,cmd)
        state = Wait_Check(data_queue, COMMAND, Sum)
        if state ==1:
            height_mode=0
            print state
            return state
def send_rcdata(rc_data):
    #print time.clock()
    rc_data=map(int,rc_data)
    print rc_data[0:4]
    #listbox.insert(0, "rc_data")
    while(1):
        cmd,Sum= assembly_cmd(HEAD_SEND, RCDATA, rc_data)
        #out_queue.put(cmd)
        safe_put(out_queue,cmd)
        state =1# Wait_Check(data_queue, RCDATA, Sum)
        if state ==1:
            print state
            return state
        

def request_user(id=None):
    data=safe_get(data_queue)
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
    #print result
    if(id==None):
        return result
    else:
        return result[id]
