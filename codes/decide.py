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
tourqe_close = 90  #140  #open and close are revese in ur3
tourqe_open = -30
Open = 2600
Velocity = 600

DXL_MAXIMUM_POSITION_VALUE = Open
BAUDRATE = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID = 6
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
#################################################camera_call-back###############################################################################################               

def pos_callback(msg: pose):     

         print("camera call-back")
         global step, zc, kk, conveyor_mode, ghx, ghy
         xd = [0,0,0,0,0,0,0,0,0,0]
       
         xmax, ymax, dmax, zmax, omax = 0, 0, 0, 0, 0
         xs, ys, zs, os, ds = 0, 0, 0, 0, 0
         xb1 = 0         
         xb2 = 0

         
         for i in range(10):
            field_name = f'xc{i+1}'
            xd[i] = getattr(msg, field_name)    
  

         xmax = xd[0]
         ymax = xd[1]
         dmax = xd[2]
         zmax = xd[3]
         omax = xd[4]
         
#############################################Nothing detected######################################            
         if xmax == 0 and ymax == 0 and dmax == 0 and zmax == 0 and omax == 0:  
             print("nothing detected")
             print("Run !")   
             conveyor_mode = 1
             xco,yco,dco,zco,oco = 0,0,0,0,0    
             moveco=gant()                 
 
             moveco.x=xco
             moveco.y=yco 
             moveco.d=dco
             moveco.z=zco 
             moveco.o=oco 
             moveco.f = 5             
 
             gant_pub.publish(moveco)      
###################################### Something detected & conveyor is running #################################         
         else:      
          print("something detected")
          if conveyor_mode == 1:   #running

################################## if xmax is more than critical point #####################################                     
           
           if xmax > -150: #50
              print("Stop!") 
              conveyor_mode = 0  
              moveco=gant()
              moveco.x = 1
              moveco.y = 1
              moveco.d = 1
              moveco.z = 1
              moveco.o = 1
              moveco.f = 5  
              gant_pub.publish(moveco) 
              
              time.sleep(1)  #waits until conveyor stops then asks camera to have an accurate position      
              
                 
              position=decision() 
              position.fl=1
              position.fd=1         
              time.sleep(.1)
              pos_pub.publish(position)                     

            
################################### if xmax hasnot reached critical point ###############################################
           else:     #something has detected but hasnot reach critical point
             print("Run !")   
             conveyor_mode = 1
             xco,yco,dco,zco,oco = 0,0,0,0,0    
             moveco=gant()                 
 
             moveco.x=xco
             moveco.y=yco 
             moveco.z=zco 
             moveco.d=dco 
             moveco.o=oco 
             moveco.f = 5             
 
             gant_pub.publish(moveco) 
###################################### something detected & conveyor is stop ########################################             
          else:   #if conveyor is stop
               print("conveyor_mode")
               print(conveyor_mode)
########################################### go for maximum one ######################################                
                 
               print("xmax = ",xmax)
               print("ymax = ",ymax)
               print("dmax = ",dmax)
               print("omax = ",omax)
               print("zmax = ",zmax)
               
               if zmax == 0:
               
                 print ("This is a single tree:")
                 print("Press 1 or 2 or 3 to continue! (or press ESC to ask camera!)")
                 #print("Press Space to continue! (or press ESC to ask camera!)")
                 b = getch()
                 #if b == chr(32):
                 if b == chr(49) or b == chr(50) or b == chr(51): 
                   
                   xs = xmax
                   ys = ymax
                   ds = dmax
                   os = omax
                   zs = zmax
                 
                   if b == chr(49):  
                     ou = os + 1000
                   if b == chr(50):  
                     ou = os + 2000                     
                   if b == chr(51):  
                     ou = os + 3000

                   xu = xs
                   yu = ys
                   du = ds
                   zu = 0  #single
                   #ou = os
                   moveu=gant()                 

                   moveu.x=xu
                   moveu.y=yu 
                   moveu.d=du 
                   moveu.z=zu 
                   moveu.o=ou 
                   moveu.f = 1                 
  
                   gant_pub.publish(moveu) 

                   #print("single gantry")
                   xg=ghx   #goes to it`s home position                     
                   yg=ghy                 
                   zg = 0

                   move=gant() 
                   move.x=xg 
                   move.y=yg 
                   move.z=zg   

                   move.f=4
                   #zc=zg        
                   gant_pub.publish(move)  

                   time.sleep(2)                
         
                   move.f=2
                   gant_pub.publish(move)
         

                   move.f=3
                   gant_pub.publish(move)   
                   
                 elif b == chr(0x1b):  #ESC
                   print("you pressed Escape")
                   position=decision() 
                   position.fl=1
                   position.fd=1         
                   time.sleep(.1)
                   pos_pub.publish(position)  

                 else:  
                   print("you pressed something")
                   position=decision() 
                   position.fl=1
                   position.fd=1         
                   time.sleep(.1)
                   pos_pub.publish(position)    

               else: #bunch           
                             
                if ymax > 20 and ymax < 40:
                  gap = 0
                else: 
                  gap = 0  
                print ("This is a bunch of trees:")
                #print("Press Space to continue! (or press ESC to ask camera!)")
                print("Press 1 or 2 or 3 to continue! (or press ESC to ask camera!)")
                b = getch()
                #if b == chr(32):
                if b == chr(49) or b == chr(50) or b == chr(51):
                  if (omax < 20 and omax > -20):  
                  #if (omax < 120 and omax > -120):   #straight
                    print("straight")
                    xb1 = xmax
                    xb2 = ymax  #width of bunch
                    db = dmax                

                    xbc = xb1
                    xbc = xbc+40  #command ur3 left finger 
                    xbc = xbc-(xb2/2)+gap  #incide the bunch                                    
                    yu = 400 #350
                    xu = xbc    
                    du = db            
                    zu = 1
                    #ou = omax
                    if b == chr(49):
                      ou = 0.075
                      zg = 138

                    if b == chr(50):
                      ou = 0.085
                      zg = 130

                    if b == chr(51):
                      ou = 0.092     
                      zg = 122             
                    moveu=gant()                 
 
                    moveu.x=xu
                    moveu.y=yu 
                    moveu.d=du 
                    moveu.z=zu 
                    moveu.o=ou 
                    moveu.f = 1                 
  
                    gant_pub.publish(moveu)  


                    xpc = xb1
                    xpc = xpc-(xb2/2)-gap   # incide the bunch
                    xpc = xpc-35   #commands gantry finger

                    if xpc > 45:  #right limit swith
                      xpc = 45 
                    if xpc<-300:   #left limit switch
                      xpc=-300 

                    xg = xpc       
                   
                    yg = 700   #yg = 300 y is not closed loop yet 
                    #zg = 130   
              
               
                    move=gant()
             
                    move.x=xg 
                    move.y=yg 
                    move.z=zg   
         
                    move.f=2
                    gant_pub.publish(move)
         

                    move.f=3
                    gant_pub.publish(move)
                    time.sleep(1.5)
                    if i == 1 and j == 1:
                      move.f=4
                      zc=zg
                    else:
                      if xg < -20:
                         time.sleep(1.5)
  
                         move.f=4
                         zc=zg 
                      else:
                         time.sleep(2.5)
  
                         move.f=4
                         zc=zg                         
       
                    gant_pub.publish(move)  




                  else:    #oriented
                    print("oriented")
                    xb1 = xmax
                    xb2 = ymax  #width of bunch
                    db = dmax                

                    xbc = xb1 - (xb2/2)  #middle of the bunch
                                                       
                    yu = 400 #350
                    xu = xbc    
                    du = db            
                    zu = 2
                    ou = omax     

                    if b == chr(49):
                      du = 0.075
                  

                    if b == chr(50):
                      du = 0.085
                   

                    if b == chr(51):
                      du = 0.092     
                    


                    moveu=gant()                 
 
                    moveu.x=xu
                    moveu.y=yu 
                    moveu.d=du 
                    moveu.z=zu 
                    moveu.o=ou 
                    moveu.f = 1                 
  
                    gant_pub.publish(moveu)                  


                    xg=ghx   #goes to it`s home position                     
                    yg=ghy                 
                    zg = 0

                    move=gant() 
                    move.x=xg 
                    move.y=yg 
                    move.z=zg   

                    move.f=4
                    #zc=zg        
                    gant_pub.publish(move)  

                    time.sleep(2)                
                    move.f=2
                    gant_pub.publish(move)
         

                    move.f=3
                    gant_pub.publish(move)                       
                            
                elif b == chr(0x1b):  #ESC
                   position=decision() 
                   position.fl=1
                   position.fd=1         
                   time.sleep(.1)
                   pos_pub.publish(position)  

                else:  
                   position=decision() 
                   position.fl=1
                   position.fd=1         
                   time.sleep(.1)
                   pos_pub.publish(position)                         
        

    
#################################################robots_call-back#####################################################         

def gantry_callback(msg: gant) :
     global zc
     
     #print(msg.f)
     global i, j, k, s, r, g, c 
     global TORQUE_DISABLE
     global TORQUE_ENABLE 
     
     if msg.f>=1000 and msg.f<3000 :
       i = 1  
        
     if msg.f>=3000 and msg.f<4000 :
       j = 1
    
     if msg.f>=4000 and msg.f<5000 :
       k = 1
       
     if msg.f == 10 :
       r = 1       

     if msg.f == 50 :  #conveyor is stop
       c = 1  
     
     if msg.f == 51 :  #conveyor is running
         position=decision() 
         position.fl=1
         position.fd=1         
         time.sleep(.1)
         pos_pub.publish(position) 
       

     if i == 1 and j == 1 and k == 1:    

        
       
       if s==0 :
         print("oneeeeeeeeeee")
         grip(tourqe_open)   
  
         rospy.sleep(0.5)

         time.sleep(.1)
         
         move=gant()
         
         move.x=ghx
         move.y=ghy    
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
         #print("enddddddddddd")
         
       elif s==1 and r == 1 and c == 1:  #every one is at home
       
         #print("twooooooooooooo") 

         grip(tourqe_open)       
         
    
         position=decision() 
         position.fl=1
         position.fd=1         
         time.sleep(.1)
         pos_pub.publish(position)  
         i, j, k, r = 0, 0, 0, 0         
         s = 2       
         #print("twwwww") 
              
       elif s==2 and r!=1:  #when gantry reaches but ur3 doesnot

           
        if msg.f>4000:
         msg.f=msg.f-4000 
         if msg.f>110:
            print("gantry close")
            grip(tourqe_close) 
                 
       elif s==2 and r == 1:  #I added r == 1       
         #print("threeeeeeeee")     
  
         if zc>100:             #because gantry blocks the view of camera
            i, j, k= 0, 0, 0
            grip(tourqe_open)
            move=gant()   
            move.x=ghx 
            move.y=ghy
            move.z=0 
            move.f=4
            gant_pub.publish(move) 

            time.sleep(.2)

            move.f=2
            gant_pub.publish(move)        

            move.f=3
            gant_pub.publish(move)
            zc = move.z
                   
            

         if i == 1 and j == 1 and k == 1:
           position=decision() 
           position.fl=1
           position.fd=1         
           time.sleep(.1)
           pos_pub.publish(position)  
           i, j, k, r = 0, 0, 0, 0    
           
           s = 2  
           #print("tttttttttttt")

       elif s==3:   ####################this part never is used###########
         
         time.sleep(.1)
         print("fourrrrrrrrr")
         move=gant()
         
         move.x=ghx
         move.y=ghy
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
 
   i, j, k, s, r, c, g, kk= 0, 0, 0, 0, 0, 0, 0, 0
   ghx = -330  
   ghy = 850
   step, zc = 0, 0
   conveyor_mode = 0
   
   rospy.init_node('Decide', anonymous=True) 
   pos_sub = rospy.Subscriber("pose_topic",pose,callback=pos_callback)
   pos_pub = rospy.Publisher('decision_topic',decision, queue_size=10)  
   gant_sub = rospy.Subscriber("gantry_movement",gant,callback=gantry_callback)
   gant_pub = rospy.Publisher('gantry_request',gant, queue_size=10)   

   
   rospy.spin()
   
   

