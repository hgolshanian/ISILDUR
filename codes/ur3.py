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

import geometry_msgs.msg

################################
# import moveit_msgs.msg
# from moveit_msgs.msg import Constraints, JointConstraint
#################################
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

close4 = 3537  #210
close7 = 2550  #2440
tourqe_close = -110  
tourqe_open = 45
#Open = 2600
Velocity = 600

#DXL_MAXIMUM_POSITION_VALUE = close
BAUDRATE = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID = 4
DXL_IDn = 7
a = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque



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
  # Get the current joint values after moving
  current_joint_values = move_group.get_current_joint_values()
  j6_current = current_joint_values[5]  # Get the 6th joint value
  print(f"Current J6 value: {j6_current} radians")  # Print J6 value  



#pose_goal = Pose()
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur3_move',anonymous=True)
ur_pub = rospy.Publisher('gantry_movement',gant, queue_size=10)     
#group = [moveit_commander.MoveGroupCommander("manipulator")]  # ur3 moveit group name: manipulator

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)



# Set the planner
move_group.set_planner_id("Cartesian Path Planning") 

# You can also set the planning time
move_group.set_planning_time(10)  # In seconds


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


#################################################################################################
def gripp():    
  # Control table address
  ADDR_TORQUE_ENABLE = 64
  ADDR_GOAL_POSITION = 116
  ADDR_PRESENT_POSITION = 132
  dxl4_goal_position = close4  
  dxl7_goal_position = close7 
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
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 11, 3)
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, 11, 3)
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
  else:
    print("DYNAMIXEL has been successfully configured as position_Control Mode")

  # write drive_mode
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, 10, 5)
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_IDn, 10, 5)
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
  else:
    print("DYNAMIXEL has been successfully configured as velocity_control Mode")
  # write velocity
  dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, 112, Velocity)
  dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_IDn, 112, Velocity)
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
  else:
    print("Velocity has successfully changed")

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

  dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,
                                                            dxl4_goal_position)
  dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_IDn, ADDR_GOAL_POSITION,
                                                            dxl7_goal_position)  

  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
###############################################################################################

grip(tourqe_open)

############################################################################################### 

j1=(pi/180)*-16.5
j2=(pi/180)*-52
j3=(pi/180)*52.8
j4=(pi/180)*-90
j5=(pi/180)*-90
j6=(pi/180)*-16  
joint_angles = [j1, j2, j3, j4, j5, j6]
move_ur3_joint_space(joint_angles) 

print("home")

flag = gant()    
flag.f=10
ur_pub.publish(flag)  
print("ready")
    
def ur3_callback(data):

 if data.f == 1: 
  rospy.loginfo("decision_msg:(%2f,%2f,%2f,%2f,%2f",data.x,data.y,data.d,data.z,data.o)
  
   
  #####################Added for task space##################

  
  if data.x > 250:

    j1=(pi/180)*35.95
    j2=(pi/180)*-44.6
    j3=(pi/180)*58.4
    j4=(pi/180)*-103.93
    j5=(pi/180)*-90
    j6=(pi/180)*-54
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)
    
  elif data.x > 150:  
  
    j1=(pi/180)*44.5
    j2=(pi/180)*-61.3
    j3=(pi/180)*75.74
    j4=(pi/180)*-104.6
    j5=(pi/180)*-90
    j6=(pi/180)*-46
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)  
    
  elif data.x > 50:  
  
    j1=(pi/180)*55.8
    j2=(pi/180)*-70
    j3=(pi/180)*88.73
    j4=(pi/180)*-108.8
    j5=(pi/180)*-90
    j6=(pi/180)*-36
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)     
    
  elif data.x > -50:

    j1=(pi/180)*69.85
    j2=(pi/180)*-73
    j3=(pi/180)*93
    j4=(pi/180)*-110
    j5=(pi/180)*-90
    j6=(pi/180)*-20
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)    
  
  elif data.x>-150:
    j1=(pi/180)*85.2
    j2=(pi/180)*-70
    j3=(pi/180)*88.85
    j4=(pi/180)*-109
    j5=(pi/180)*-90
    j6=(pi/180)*-5
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)

  elif data.x > -250:
    j1=(pi/180)*100
    j2=(pi/180)*-61.3
    j3=(pi/180)*75.8
    j4=(pi/180)*-104.6
    j5=(pi/180)*-90
    j6=(pi/180)*10
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)    
    
  else: 
    
    j1=(pi/180)*112.5
    j2=(pi/180)*-46.5
    j3=(pi/180)*50.9
    j4=(pi/180)*-94.5
    j5=(pi/180)*-90
    j6=(pi/180)*22.5
    joint_angles = [j1, j2, j3, j4, j5, j6]
    move_ur3_joint_space(joint_angles)  
    
  pose_target = geometry_msgs.msg.Pose()
  yr=(data.y)/1000 
  xr=(data.x)/1000
  
  if data.z == 2:
    #zb = 0.073  
    zb = data.d
    print("zb=")
    print(zb)
    ori = data.o

  if data.z == 1:
    zb = data.o  #data.o is in m

    print("zb=")
    print(zb)
    ori = 0
    
  if data.z == 0:
    gripp() 
    ori = data.o 
    if data.o >= 500 and data.o < 1500:
      ori = data.o - 1000
      zs = 0.072 #0.074

    if data.o >= 1500 and data.o < 2500:
      ori = data.o - 2000
      zs = 0.074 + 0.003

    if data.o >= 2500:
      ori = data.o - 3000
      zs = 0.074 + 0.006    
    
   
    print("zs=")
    print(zs)

  #Get the current pose of the end effector
  current_pose = move_group.get_current_pose().pose
    
  # Create a list of waypoints
  waypoints = []


  if data.y > 300 :
    print("current_pose.position.z:")
    print(current_pose.position.z)
    # Convert Euler angles to quaternion (unchanged)
    euler = (pi / 180) * -180, 0, (pi / 180) * ori
    quaternion = tf.transformations.quaternion_from_euler(*euler)

    # Set the orientation using quaternion (unchanged)
    pose_target1 = geometry_msgs.msg.Pose()
    pose_target1.orientation.x = quaternion[0]
    pose_target1.orientation.y = quaternion[1]
    pose_target1.orientation.z = quaternion[2]
    pose_target1.orientation.w = quaternion[3]

    pose_target1.position.x = current_pose.position.x
    pose_target1.position.y = current_pose.position.y
    pose_target1.position.z = 0.15
    waypoints.append(pose_target1)

    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = xr
    pose_target.position.y = yr
    pose_target.position.z = 0.15
    pose_target.orientation = pose_target1.orientation
    waypoints.append(pose_target)


  else:
     # Convert Euler angles to quaternion (unchanged)
    euler = (pi / 180) * -180, 0, (pi / 180) * ori
    quaternion = tf.transformations.quaternion_from_euler(*euler)

    # Set the orientation using quaternion (unchanged)
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]



    # Set the initial position based on the received data
    pose_target.position.x = xr
    pose_target.position.y = yr  
    pose_target.position.z = current_pose.position.z
    waypoints.append(pose_target)  # Add this as the first waypoint

  pose_target2 = geometry_msgs.msg.Pose()
  pose_target2.position.x = pose_target.position.x  # Keep X from the previous target
  pose_target2.position.y = pose_target.position.y  # Keep Y from the previous target
  pose_target2.position.z = 0.14 
  pose_target2.orientation = pose_target.orientation 
       
  waypoints.append(pose_target2)


  pose_target3 = geometry_msgs.msg.Pose()
  # Modify pose based on conditions (data.z, etc.)
  if data.z == 0:  # For single tree
      pose_target3.position.z = zs  # Lower the Z-coordinate for the tree
  elif data.z == 1 or data.z == 2:
      pose_target3.position.z = zb  # Adjust the Z-coordinate for multi-tree
      if xr > 0.05:
          rospy.sleep(0.5)  # Wait for gantry if necessary
  pose_target3.position.x = pose_target2.position.x  # Keep X from the previous target
  pose_target3.position.y = pose_target2.position.y  # Keep Y from the previous target
  pose_target3.orientation = pose_target2.orientation

  # Add the modified target pose to waypoints
  waypoints.append(pose_target3)

  # Plan the Cartesian path (e.g., 1 cm resolution with no jump threshold)
  (plan,fraction) = move_group.compute_cartesian_path(
      waypoints,   # waypoints to follow
      0.01,        # eef_step: the resolution of the path in meters
      3.14          # jump_threshold: threshold for preventing large jumps in joint space
  )
  
# Execute the plan
  if fraction > 0.9:  # Only execute if most of the path was planned successfully
        move_group.execute(plan, wait=True)  
  
  #move_group.execute(plan, wait=True)

  # Stop and clear targets after execution
  move_group.stop()
  move_group.clear_pose_targets()    



  rospy.sleep(.1)
    
  ################################
  grip(tourqe_close)
  ################################
  rospy.sleep(.7)

  #####################Realeasing point##################
 
  

  if data.z == 0: #singletree
     print("single")

     j1=(pi/180)*57
     j2=(pi/180)*-52.7
     j3=(pi/180)*35.3
     j4=(pi/180)*-72.7
     j5=(pi/180)*-90
     j6=(pi/180)*-33.16  
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles) 
  
     move_ur3_joint_space(joint_angles)
     j6=j6+(pi/180)*90
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)
     j6=j6-(pi/180)*90
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)
     j5=j5+(pi/180)*50
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)
     j5=j5-(pi/180)*50
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)
     j5=j5-(pi/180)*50
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)                     
     j5=j5+(pi/180)*50
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)     



     j1=(pi/180)*-102
     j2=(pi/180)*-36
     j3=(pi/180)*48
     j4=(pi/180)*-101
     j5=(pi/180)*-90
     j6=(pi/180)*-12  
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles) 
     #######################################
     grip(tourqe_open)
     #######################################
 
     rospy.sleep(.3)  
     #go to home
     j1=(pi/180)*-16.5
     j2=(pi/180)*-52
     j3=(pi/180)*52.8
     j4=(pi/180)*-90
     j5=(pi/180)*-90
     j6=(pi/180)*-16  
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)   
 
 
     flag = gant()    
     flag.f=10
     ur_pub.publish(flag)      


    
  #Create a list of waypoints
  waypoints = []

  if data.z == 1:     #bunch
     print("straight bunch")
     
     
     euler = (pi / 180) * -180, 0, (pi / 180) * 0
     quaternion = tf.transformations.quaternion_from_euler(*euler)

     # Set the orientation using quaternion (unchanged)
     pose_target = geometry_msgs.msg.Pose()
     pose_target.orientation.x = quaternion[0]
     pose_target.orientation.y = quaternion[1]
     pose_target.orientation.z = quaternion[2]
     pose_target.orientation.w = quaternion[3]

     pose_target.position.x = xr + 0.2
     pose_target.position.y = yr     
     pose_target.position.z = 0.130
     waypoints.append(pose_target)  # Add this as the first waypoint
  
     
     # Plan the Cartesian path (e.g., 1 cm resolution with no jump threshold)
     (plan,fraction) = move_group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # eef_step: the resolution of the path in meters
        3.14          # jump_threshold: threshold for preventing large jumps in joint space
     )
  
     # Execute the plan
     if fraction > 0.9:  # Only execute if most of the path was planned successfully
        move_group.execute(plan, wait=True)  
  
     #move_group.execute(plan, wait=True)

     # Stop and clear targets after execution
     move_group.stop()
     move_group.clear_pose_targets()  

    #####################################
     grip(tourqe_open)
    #####################################
 
     rospy.sleep(.3)  
     #now we are sure closing robot grippers doesnot have overlap
     flag = gant()    
     flag.f=10
     ur_pub.publish(flag) 

     j1=(pi/180)*57
     j2=(pi/180)*-52.7
     j3=(pi/180)*35.3
     j4=(pi/180)*-72.7
     j5=(pi/180)*-90
     j6=(pi/180)*-33.16  
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)      

     #go to home
     j1=(pi/180)*-16.5
     j2=(pi/180)*-52
     j3=(pi/180)*52.8
     j4=(pi/180)*-90
     j5=(pi/180)*-90
     j6=(pi/180)*-16  
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles) 


  if data.z == 2:

     euler = (pi / 180) * -180, 0, (pi / 180) * 0  # Euler angles (fi, te, si)
     quaternion = tf.transformations.quaternion_from_euler(*euler)
     
     pose_target.position.x = xr
     pose_target.position.y = yr
     pose_target.position.z =zb
     pose_target.orientation.x = quaternion[0]
     pose_target.orientation.y = quaternion[1]
     pose_target.orientation.z = quaternion[2]
     pose_target.orientation.w = quaternion[3]      
     move_group.set_pose_target(pose_target)
     plan = move_group.go(wait=True)
     move_group.stop()
     move_group.clear_pose_targets()  

     rospy.sleep(.3) 

    #####################################
     grip(tourqe_open)
    #####################################
      
     rospy.sleep(.3)  
     pose_target.position.z = 0.25

     move_group.set_pose_target(pose_target)
     plan = move_group.go(wait=True)
     move_group.stop()
     move_group.clear_pose_targets() 

     #go to home
     j1=(pi/180)*-16.5
     j2=(pi/180)*-52
     j3=(pi/180)*52.8
     j4=(pi/180)*-90
     j5=(pi/180)*-90
     j6=(pi/180)*-16  
     joint_angles = [j1, j2, j3, j4, j5, j6]
     move_ur3_joint_space(joint_angles)    
 
     flag = gant()    
     flag.f=10
     ur_pub.publish(flag)   

 
  
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


