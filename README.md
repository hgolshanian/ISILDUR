# ISILDUR
This project is about developing a closed-loop system that integrates robotic manipulators with a deep learning-based visual object recognition and pose estimation system for singulating tree saplings. Robotic manipulators, including a gantry robot and an elbow manipulator, emulate human hand movements. A custom dataset of annotated images was created and augmented to train the YOLOv8 model for localizing individual trees and tree bunches. The system operates in real-time within a ROS-based closed-loop framework, ensuring seamless interaction between visual feedback and robotic actions.

![image](https://github.com/user-attachments/assets/04f865a1-2a37-4963-a2a7-e793cddb84c8)

Figure1.Flowchart of the singulation algorithm.

To bring up with the elbow manipulator, which is a UR3 robot, we need to run both the robot and MoveIt in the command line using the following commands:
##
    roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.100
##
    roslaunch ur3e_moveit_config moveit_planning_execution.launch limited:=true
    
We use ROS Noetic to establish a closed-loop system integrating the RealSense camera, UR3 robot, gantry robot, and conveyor belt. Additionally, a decision node is implemented to coordinate and control the interactions between these components.

<div style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/e20ad3ab-51f4-4e5f-8c75-d209c12f5b43" alt="Picture1" width="300" height="300"/>
</div>

Figure2. System setup

To bring up each node we need to run the related python code For example to bring up the realsense camera:
##
    rosrun camera camera.py
