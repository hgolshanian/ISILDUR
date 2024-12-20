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
    
frames = [[0] * 50 for _ in range(10)] ##############
def pos_callback(msg: pos):   
  global des_data
  global d,k
  
  xc = [0,0,0,0,0,0,0,0,0,0]
  yc = [0,0,0,0,0,0,0,0,0,0]
  zc = [0,0,0,0,0,0,0,0,0,0]
  dc = [0,0,0,0,0,0,0,0,0,0]
  oc = [0,0,0,0,0,0,0,0,0,0]



  for i in range(10):
        field_name = f'x{i+1}'
        xc[i] = getattr(msg, field_name)
        
  for i in range(10):  
        field_name = f'y{i+1}'
        yc[i] = getattr(msg, field_name)        
        
  for i in range(10):  
        field_name = f'z{i+1}'
        zc[i] = getattr(msg, field_name)

  for i in range(10):  
        field_name = f'd{i+1}'
        dc[i] = getattr(msg, field_name)         
        
  for i in range(10):  
        field_name = f'o{i+1}'
        oc[i] = getattr(msg, field_name)    
  
  # Check if  values are not valid
  for i in range(10):        
        if xc[i] == 999:
            print("Not valid!")
            return  # Exit the function if any value is 999  


   
  for i in range(10):
        frames[k][i * 5] = xc[i]
        frames[k][i * 5 + 1] = yc[i]
        frames[k][i * 5 + 2] = zc[i]
        frames[k][i * 5 + 3] = dc[i]
        frames[k][i * 5 + 4] = oc[i]       


  
  k=k+1  
  if k==10: ##############
    k=0

  
 
  # Find the largest x across all frames, ignoring all-zero arrays
  max_x = -float('inf')
  maxt = None

  for frame in frames:
        for i in range(10):
            x = frame[i * 5]
            y = frame[i * 5 + 1]
            z = frame[i * 5 + 2]
            d = frame[i * 5 + 3]
            o = frame[i * 5 + 4]

            # Check if all values in this set are zero
            if x == 0 and y == 0 and z == 0 and d == 0 and o == 0:
                continue  # Ignore this set if all values are zero

            # Update max_x and related values if a larger x is found
            if x > max_x:
                max_x = x
                maxt = (x, y, z, d, o)


        print(frame)
  if maxt is None:  #if all the arrays in all frames are zero
        maxt = (0, 0, 0, 0, 0)

  
  largest_x, largest_y, largest_z, largest_d, largest_o = maxt
  print(f"Largest :")
  print(f" x = {largest_x}, y = {largest_y}, z = {largest_z}, d = {largest_d}, o = {largest_o}")

 ###########################################################################################     

  if des_data is not None:  #I changed the position this code because it did not work 
    if des_data.fl == 1: 
         
      if des_data.fd == 1:  
         d=1
         print("yesssssssssssssssssssssssssss")   


  if d==1: 
    print("sennnnnddddddddddddddddd")
    position=pose() 

  

    xc[0] = largest_x
    xc[1] = largest_y
    xc[2] = largest_d
    xc[3] = largest_z
    xc[4] = largest_o
        

    for i, value in enumerate(xc):
          setattr(position, f'xc{i + 1}', value)               
        
    pos_pub.publish(position)
    print(f"Published position: xc1={position.xc1}, yc1={position.yc1}, dc1={position.dc1}, zc1={position.zc1}, oc1={position.oc1}")

    d=0 
    des_data = None
    
             
                                     
   
if __name__ == '__main__':  
   
   d=0 
   k=0
   rospy.init_node('Decide', anonymous=True)
   pos_sub = rospy.Subscriber("pos_topic",pos,callback=pos_callback)
   pos_pub = rospy.Publisher('pose_topic',pose, queue_size=10)
   pos_sub = rospy.Subscriber("decision_topic",decision,callback=des_callback)

   rospy.spin()
   
   
   
   
   
