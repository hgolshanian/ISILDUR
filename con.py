#!/usr/bin/env python

import rospy
import sys
from camera.msg import pos
from Decision.msg import decision 
from connector.msg import pose 


global des_data
des_data = None


def des_callback(msge: decision):
    global des_data
    des_data = msge
    

def pos_callback(msg: pos):   
  global des_data
  global d
  
  xc = [0,0,0,0,0,0,0,0,0,0]
  yc = [0,0,0,0,0,0,0,0,0,0]
  zc = [0,0,0,0,0,0,0,0,0,0]
    
    
  if d==1: 
  
    for i in range(10):
        field_name = f'x{i+1}'
        xc[i] = getattr(msg, field_name)
        
    for i in range(10):  
        field_name = f'y{i+1}'
        yc[i] = getattr(msg, field_name)        
        
    for i in range(10):  
        field_name = f'z{i+1}'
        zc[i] = getattr(msg, field_name)
  
      
    position=pose() 
    
    for i, value in enumerate(xc):
        setattr(position, f'xc{i + 1}', value)
        
    for i, value in enumerate(yc):
        setattr(position, f'yc{i + 1}', value)        
     
    for i, value in enumerate(zc):
        setattr(position, f'zc{i + 1}', value)
        
    pos_pub.publish(position)
   
    d=0 
    des_data = None
    
  if des_data is not None:
    if des_data.fl == 1: 
         
      if des_data.fd == 1:  
         d=1
         print("yesssssssssssssssssssssssssss")                
                                     
   
if __name__ == '__main__':  
   
   d=0  # ur3 and gantry determine first movement 

   rospy.init_node('Decide', anonymous=True)
   pos_sub = rospy.Subscriber("pos_topic",pos,callback=pos_callback)
   pos_pub = rospy.Publisher('pose_topic',pose, queue_size=10)
   pos_sub = rospy.Subscriber("decision_topic",decision,callback=des_callback)

   rospy.spin()
   
   
   
   
   
