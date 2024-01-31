#!/usr/bin/env python

import rospy
import sys
from camera.msg import pos
from Decision.msg import decision  
from connector.msg import pose 
from gant.msg import gant

global zc 
#---------------------dynamixel---------------------------------#

import os

if os.name == 'nt':
  import msvcrt


  def getch():
    return msvcrt.getch().decode()
else:
  import sys, tty, termios

  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)


  def getch():
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ********* DYNAMIXEL Model definition *********
MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430

# close = 1100
tourqe_close = 140  #open and close are revese in ur3
tourqe_open = -30
Open = 2600
Velocity = 600

DXL_MAXIMUM_POSITION_VALUE = Open
BAUDRATE = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID = 4
DXL_IDn = 5
a = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque

# dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]    # Goal position
dxl_goal_position = DXL_MAXIMUM_POSITION_VALUE  # Goal position

portHandler = PortHandler(DEVICENAME)

packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
  print("Succeeded to open the port")
else:
  print("Failed to open the port")
  print("Press any key to terminate...")
  getch()
  quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
  print("Succeeded to change the baudrate")
else:
  print("Failed to change the baudrate")
  print("Press any key to terminate...")
  getch()
  quit()
#---------------------dynamixel---------------------------------#   
def grip(mode):
  
   # Control table address
   ADDR_TORQUE_ENABLE = 64
   ADDR_GOAL_Current = 102  # tourqe
   ADDR_PRESENT_Current = 126  # tourqe
   DXL_Current_VALUE1 = mode  # tourqe
   # Disable Dynamixel Torque
   dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                          TORQUE_DISABLE)
                                                          
   dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, ADDR_TORQUE_ENABLE,
                                                          TORQUE_DISABLE)                                                          
                                                          
   if dxl_comm_result != COMM_SUCCESS:
     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
     print("%s" % packetHandler.getRxPacketError(dxl_error))

   # write operating_mode
   dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 11, 0)
   dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, 11, 0)   
   if dxl_comm_result != COMM_SUCCESS:
     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
     print("%s" % packetHandler.getRxPacketError(dxl_error))
   else:
     print("DYNAMIXEL has been successfully configured as Current_Control Mode")

   # Enable Dynamixel Torque
   dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                          TORQUE_ENABLE)
   dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, ADDR_TORQUE_ENABLE,
                                                          TORQUE_ENABLE)                                                          
                                                          
   if dxl_comm_result != COMM_SUCCESS:
     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
     print("%s" % packetHandler.getRxPacketError(dxl_error))
   else:
     print("Dynamixel has been successfully connected")

   dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_Current,
                                                          DXL_Current_VALUE1)
   dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_IDn, ADDR_GOAL_Current,
                                                          DXL_Current_VALUE1)                                                          
   if dxl_comm_result != COMM_SUCCESS:
     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
   elif dxl_error != 0:
     print("%s" % packetHandler.getRxPacketError(dxl_error))


grip(tourqe_open)


global i, j, k, r

import time
#################################################camera_call-back##########################################################               

def pos_callback(msg: pose):     

         global step, zc 
         xd = [0,0,0,0,0,0,0,0,0,0]
         yd = [0,0,0,0,0,0,0,0,0,0]
         zd = [0,0,0,0,0,0,0,0,0,0] 
         xmin = 0
         xmax = 0 
         xb1 = 0
         xb2 = 0
         fb = 0
         
         for i in range(10):
            field_name = f'xc{i+1}'
            xd[i] = getattr(msg, field_name)
        
         for i in range(10):  
            field_name = f'yc{i+1}'
            yd[i] = getattr(msg, field_name)        
        
         for i in range(10):  
            field_name = f'zc{i+1}'
            zd[i] = getattr(msg, field_name) 
            
         for i in range(10):
         
            if i==0:
               xmin = xd[i] 
               ymin = yd[i] 
               zmin = zd[i]
               xmax = xd[i]
               ymax = yd[i]
               zmax = zd[i]
               
               if zd[i] == 1: #rootb
                 xb1=xd[i]
                 xb2=yd[i]
                 fb=1
                 print("first")
              
            else:
              if zd[i] == 1:  #rootb
                 xb1=xd[i]
                 xb2=yd[i] 
                 fb=1 
                 print("not-first")            
            
              if xd[i] != 0:  #the object may be less than 10 (for ur3 root or rootb)          
                if xd[i]<xmin:
                  xmin = xd[i]
                  ymin = yd[i]
                  zmin = zd[i]
                if xd[i]>xmax:
                  xmax = xd[i]
                  ymax = yd[i]
                  zmax = zd[i]
           
         
       #  if xmin == 0:  #no trees detected
       #    xminc, yminc, zminc = 0, 0, 0
          
         if fb==1:  #it is a bunch
                    
           xb = ((xb2+xb1)/2)
           xbc = (xb*-1.58) + 222 
           xbc = xbc+50  #command ur3 left finger 
           xu = xbc
           yu = xb2    #we donot use it in ur3 code
           zu = 1  #it is a bunch         
         
         else:  #it is not bunch       
         
           xminc=(xmin*-1.58) + 222
           yminc=(ymin*-1.56)+ 970   #we donot use it in ur3 code
           zminc=zmin      
           xu = xminc
           yu = yminc
           zu = zminc  #zu = 0 means it is not a bunch
                              
         moveu=gant()                 
 
         moveu.x=xu
         moveu.y=yu 
         moveu.z=zu  
         moveu.f = 1                 
 
         gant_pub.publish(moveu) 
         
         #gantry callibration
         #xmaxc=(xmax*1.56)-561
         #ymaxc=(ymax*-1.58)+715              
       
         
         #xpc = 0    #gantry openloop
         yp = 300  #130
         
         #gantry closedloop
         if fb == 1:        #there is a bunch
           print("bunch")
           xp = ((xb2+xb1)/2)
           xpc = (xp*1.56)-561 
           xpc = xpc+50   #commands gantry right finger
           
           if xpc < 0:  #right limit swith
             xpc = 0 
           if xpc>310:   #left limit switch
             xpc=310 
           xg = xpc       
           yp = 300  #130   
           yg = yp   #yg = 300  
           zg = 245  
           
         else:       #there is a single tree
           print("single")
           xpc=300   #goes to it`s middle position
           xg = xpc 
           if xg < 0:
              xg = 0
           yp=300     #130
           yg = yp 
           z=0 
           zg = z           
        
              
         move=gant()
         
         if zc>220:
           grip(tourqe_open)
           move.z=150
           zc=move.z 
           move.f=4
           gant_pub.publish(move) 
          
       
           
         move.x=xg 
         move.y=yg 
         move.z=zg   
         
         move.f=2
         gant_pub.publish(move)
         

         move.f=3
         gant_pub.publish(move)
         
         time.sleep(2)

         move.f=4
         zc=zg 
       
         gant_pub.publish(move) 
                   
         
#################################################robots_call-back#####################################################         

def gantry_callback(msg: gant) :
     global zc
     
     print(msg.f)
     global i, j, k, s, r, g 
     global TORQUE_DISABLE
     global TORQUE_ENABLE 
     
     if msg.f>=2000 and msg.f<3000 :
       i = 1  
        
     if msg.f>=3000 and msg.f<4000 :
       j = 1
    
     if msg.f>=4000 and msg.f<5000 :
       k = 1
       
     if msg.f == 10 :
       r = 1       
     
     if i == 1 and j == 1 and k == 1:
       
       #i, j, k = 0, 0, 0
        
       
       if s==0 :
         print("oneeeeeeeeeee")
         grip(tourqe_open)   
  
         rospy.sleep(0.5)

         time.sleep(.1)
         
         move=gant()
         
         move.x=300
         move.y=130
         move.z=0
         
         move.f=4
         gant_pub.publish(move)          
         
         time.sleep(1)
         
         move.f=2
         gant_pub.publish(move)
         

         move.f=3
         gant_pub.publish(move)     
                  
         s = 1          
         i, j, k = 0, 0, 0
         print("enddddddddddd")
         
       elif s==1 and r == 1:
       
         print("twooooooooooooo") 

         grip(tourqe_open)       
         
    
         position=decision() 
         position.fl=1
         position.fd=1         
         time.sleep(.1)
         pos_pub.publish(position)  
         i, j, k, r = 0, 0, 0, 0         
         s = 2       
         print("twwwww") 
              
       elif s==2 and r!=1:  #when gantry reaches but ur3 doesnot

           
        if msg.f>4000:
         msg.f=msg.f-4000 
         if msg.f>200:
            grip(tourqe_close) 
                 
       elif s==2 and r == 1:  #I added r == 1       
         print("threeeeeeeee")     
  
         #rospy.sleep(.1)
       
         position=decision() 
         position.fl=1
         position.fd=1         
         time.sleep(.1)
         pos_pub.publish(position)  
         i, j, k, r = 0, 0, 0, 0    
           
         s = 2  #changggggggggggggggggggggggg
         print("tttttttttttt")
       elif s==3: 
         
         time.sleep(.1)
         print("fourrrrrrrrr")
         move=gant()
         
         move.x=300
         move.y=300
         move.z=245  
         move.f=4
         gant_pub.publish(move)          
         
         time.sleep(1)
         
         move.f=2
         gant_pub.publish(move)
         

         move.f=3
         gant_pub.publish(move)     
         
         i, j, k, r = 0, 0, 0, 0
         s = 0    
       
  
if __name__ == '__main__': 
 
   i, j, k, s, r, g= 0, 0, 0, 0, 0, 0
   step, zc = 0, 0
   
   rospy.init_node('Decide', anonymous=True) 
   pos_sub = rospy.Subscriber("pose_topic",pose,callback=pos_callback)
   pos_pub = rospy.Publisher('decision_topic',decision, queue_size=10)  
   gant_sub = rospy.Subscriber("gantry_movement",gant,callback=gantry_callback)
   gant_pub = rospy.Publisher('gantry_request',gant, queue_size=10)   

   
   rospy.spin()
   
   
