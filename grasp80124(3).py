#!/usr/bin/env python

# Author: Hyeonjun Park, Ph.D. candidate
# Affiliation: Human-Robot Interaction LAB, Kyung Hee University, South Korea
# koreaphj91@gmail.com
# init: 9 Apr 2019
# revision: 17 Feb 2020

import math
import sys
import rospy
import tf
import moveit_commander  # https://answers.ros.org/question/285216/importerror-no-module-named-moveit_commander/
import random
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi
import moveit_msgs.msg
from sensor_msgs.msg import JointState

#added for control
from Decision.msg import decision  
from gant.msg import gant

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
# ***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'  # X330 (5.0 V recommended), X430, X540, 2X430

# close = 1100
tourqe_close = -160
tourqe_open = 30
Open = 2600
Velocity = 600

DXL_MAXIMUM_POSITION_VALUE = Open
BAUDRATE = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID = 3
DXL_IDn = 6
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
def move_ur3_joint_space(joint_angles):
  # Initialize moveit_commander and rospy node
  moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('move_ur3_joint_space', anonymous=True)
  # Set up moveit_commander objects for planning and executing robot motions
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group_name = "manipulator"
  move_group = moveit_commander.MoveGroupCommander(group_name)
  # Set the UR3 robot arm to its initial pose
  # move_group.set_named_target("home")
  # plan = move_group.go()
  # Set the target joint angles and move the robot
  move_group.set_joint_value_target(joint_angles)
  plan = move_group.go(wait=True)



pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move',anonymous=True)
ur_pub = rospy.Publisher('gantry_movement',gant, queue_size=10)     
group = [moveit_commander.MoveGroupCommander("manipulator")]  # ur3 moveit group name: manipulator

xx = 1
################################open########################
# Control table address
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_Current = 102  # tourqe
ADDR_PRESENT_Current = 126  # tourqe
DXL_Current_VALUE1 = tourqe_open  # tourqe
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
                                                          DXL_Current_VALUE1*-1)                                                          
if dxl_comm_result != COMM_SUCCESS:
  print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
  print("%s" % packetHandler.getRxPacketError(dxl_error))

#############################################################
#while not rospy.is_shutdown(): 

j1=(pi/180)*58.7
j2=(pi/180)*-80.5
j3=(pi/180)*77.4
j4=(pi/180)*-87
j5=(pi/180)*-90
j6=(pi/180)*-32
joint_angles = [j1, j2, j3, j4, j5, j6]
move_ur3_joint_space(joint_angles)

print("home")

flag = gant()    
flag.f=10
ur_pub.publish(flag)  
print("ready")
    
def ur3_callback(data):

 if data.f == 1: 
  rospy.loginfo("decision_msg:(%2f,%2f,%2f)",data.x,data.y,data.z)
  

  
  j1=(pi/180)*58.7
  j2=(pi/180)*-80.5
  j3=(pi/180)*77.4
  j4=(pi/180)*-87
  j5=(pi/180)*-90
  j6=(pi/180)*-32
  joint_angles = [j1, j2, j3, j4, j5, j6]
  move_ur3_joint_space(joint_angles)

  
  #j1=(pi/180)*104
  #j2=(pi/180)*-67
  #j3=(pi/180)*55
  #j4=(pi/180)*-77
  #j5=(pi/180)*-90
  #j6=(pi/180)*14
  #joint_angles = [j1, j2, j3, j4, j5, j6]
  #move_ur3_joint_space(joint_angles)  
  


  #####################Added for task space##################

  if data.z == 1: #it is rootb
   yr=(data.y)/1000 

   xr=(data.x)/1000
  
   zr= (data.z)/1000
  
   fi=(pi/180)*-179.9
   te=(pi/180)*-35
   si=0
   pose_goal.orientation.w = math.cos (fi/2)*math.cos (te/2)*math.cos (si/2)+math.sin (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.x = math.sin (fi/2)*math.cos (te/2)*math.cos (si/2)-math.cos (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.y = math.cos (fi/2)*math.sin (te/2)*math.cos (si/2)+math.sin (fi/2)*math.cos (te/2)*math.sin (si/2)
   pose_goal.orientation.z = math.cos (fi/2)*math.cos (te/2)*math.sin (si/2)+math.sin (fi/2)*math.sin (te/2)*math.cos (si/2)
   pose_goal.position.x = xr+0.25
   pose_goal.position.y = 0.35  #I changed yr for trees
   #pose_goal.position.y = yr
   pose_goal.position.z = 0.18
   group[0].set_pose_target(pose_goal)
   group[0].go(True) 
  

   pose_goal.orientation.w = math.cos (fi/2)*math.cos (te/2)*math.cos (si/2)+math.sin (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.x = math.sin (fi/2)*math.cos (te/2)*math.cos (si/2)-math.cos (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.y = math.cos (fi/2)*math.sin (te/2)*math.cos (si/2)+math.sin (fi/2)*math.cos (te/2)*math.sin (si/2)
   pose_goal.orientation.z = math.cos (fi/2)*math.cos (te/2)*math.sin (si/2)+math.sin (fi/2)*math.sin (te/2)*math.cos (si/2)
   pose_goal.position.x = xr+.05
   pose_goal.position.y = 0.35   #I changed yr for trees
   #pose_goal.position.y = yr
   pose_goal.position.z = 0.075
   group[0].set_pose_target(pose_goal)
   group[0].go(True) 
  
  if data.z == 0: #it is root
  
   yr=(data.y)/1000 

   xr=((data.x))/1000
  
   zr= (data.z)/1000
  
   fi=(pi/180)*-178
   te=0
   si=0
   pose_goal.orientation.w = math.cos (fi/2)*math.cos (te/2)*math.cos (si/2)+math.sin (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.x = math.sin (fi/2)*math.cos (te/2)*math.cos (si/2)-math.cos (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.y = math.cos (fi/2)*math.sin (te/2)*math.cos (si/2)+math.sin (fi/2)*math.cos (te/2)*math.sin (si/2)
   pose_goal.orientation.z = math.cos (fi/2)*math.cos (te/2)*math.sin (si/2)+math.sin (fi/2)*math.sin (te/2)*math.cos (si/2)
   pose_goal.position.x = xr
   pose_goal.position.y = 0.35  #I changed yr for trees
   #pose_goal.position.y = yr
   pose_goal.position.z = 0.18
   group[0].set_pose_target(pose_goal)
   group[0].go(True) 
  

   pose_goal.orientation.w = math.cos (fi/2)*math.cos (te/2)*math.cos (si/2)+math.sin (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.x = math.sin (fi/2)*math.cos (te/2)*math.cos (si/2)-math.cos (fi/2)*math.sin (te/2)*math.sin (si/2)
   pose_goal.orientation.y = math.cos (fi/2)*math.sin (te/2)*math.cos (si/2)+math.sin (fi/2)*math.cos (te/2)*math.sin (si/2)
   pose_goal.orientation.z = math.cos (fi/2)*math.cos (te/2)*math.sin (si/2)+math.sin (fi/2)*math.sin (te/2)*math.cos (si/2)
   pose_goal.position.x = xr
   pose_goal.position.y = 0.35   #I changed yr for trees
   #pose_goal.position.y = yr
   pose_goal.position.z = 0.075
   group[0].set_pose_target(pose_goal)
   group[0].go(True)   
  

  rospy.sleep(.1)
  
  
  # Disable Dynamixel Torque
  TORQUE_ENABLE = 1  # Value for enabling the torque
  TORQUE_DISABLE = 0  # Value for disabling the torque
  ADDR_TORQUE_ENABLE = 64
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE,
                                                            TORQUE_DISABLE)
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, ADDR_TORQUE_ENABLE,
                                                            TORQUE_DISABLE)                                                            
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
  # Control table address
  ADDR_TORQUE_ENABLE = 64
  ADDR_GOAL_Current = 102  # tourqe
  ADDR_PRESENT_Current = 126  # tourqe
  DXL_Current_VALUE2 = tourqe_close  # tourqe
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
                                                            DXL_Current_VALUE2)
  dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_IDn, ADDR_GOAL_Current,
                                                            DXL_Current_VALUE2*-1)                                                            
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
  
  
  rospy.sleep(.2)

  #####################Added for task space##################
 
  j1=(pi/180)*52.45
  j2=(pi/180)*-58
  j3=(pi/180)*76
  j4=(pi/180)*-108
  j5=(pi/180)*-90
  j6=(pi/180)*-38
  joint_angles = [j1, j2, j3, j4, j5, j6]
  move_ur3_joint_space(joint_angles) 
 
 
 
  j1=(pi/180)*52.45
  j2=(pi/180)*-58
  j3=(pi/180)*76
  j4=(pi/180)*-108
  j5=(pi/180)*-90
  j6=(pi/180)*-38
  joint_angles = [j1, j2, j3, j4, j5, j6]
  move_ur3_joint_space(joint_angles) 
  
  
  #fi=(pi/180)*-179.9
  #te=(pi/180)*0
  #si=0
 # pose_goal.orientation.w = math.cos (fi/2)*math.cos (te/2)*math.cos (si/2)+math.sin (fi/2)*math.sin (te/2)*math.sin (si/2)
 # pose_goal.orientation.x = math.sin (fi/2)*math.cos (te/2)*math.cos (si/2)-math.cos (fi/2)*math.sin (te/2)*math.sin (si/2)
 # pose_goal.orientation.y = math.cos (fi/2)*math.sin (te/2)*math.cos (si/2)+math.sin (fi/2)*math.cos (te/2)*math.sin (si/2)
  #pose_goal.orientation.z = math.cos (fi/2)*math.cos (te/2)*math.sin (si/2)+math.sin (fi/2)*math.sin (te/2)*math.cos (si/2)
 # pose_goal.position.x = 0.2
 # pose_goal.position.y = 0.35
  #pose_goal.position.z = 0.12
 ## group[0].set_pose_target(pose_goal)
  #group[0].go(True) 
  


  #######################################
  # Control table address
  ADDR_TORQUE_ENABLE = 64
  ADDR_GOAL_Current = 102  # tourqe
  ADDR_PRESENT_Current = 126  # tourqe
  DXL_Current_VALUE1 = tourqe_open  # tourqe
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
                                                            DXL_Current_VALUE1*-1)                                                            
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
  #######################################
 
  rospy.sleep(.1)
   
  j1=(pi/180)*58.7
  j2=(pi/180)*-80.5
  j3=(pi/180)*77.4
  j4=(pi/180)*-87
  j5=(pi/180)*-90
  j6=(pi/180)*-32
  joint_angles = [j1, j2, j3, j4, j5, j6]
  move_ur3_joint_space(joint_angles) 
  
 
  flag = gant()    
  flag.f=10
  ur_pub.publish(flag)





'''
pose_goal.orientation.w = 0.0
pose_goal.position.x = 0.4 # red line      0.2   0.2
pose_goal.position.y = 0.15  # green line  0.15   0.15
pose_goal.position.z = 0.5  # blue line   # 0.35   0.6
group[0].set_pose_target(pose_goal)
group[0].go(True)

'''

  
 
  
if __name__ == '__main__': 
  
   ur_sub = rospy.Subscriber("gantry_request",gant,callback=ur3_callback)      
   rospy.spin() #if you recieved data do this loop we donot have this in publisher
   
   
       
# Shutdown moveit_commander and rospy node
moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)
rospy.signal_shutdown("Done")

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

