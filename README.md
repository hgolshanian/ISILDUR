# ISILDUR
## 🔍 Overview
This project develops a **closed-loop system** integrating robotic manipulators with a deep learning-based visual object recognition and pose estimation system, designed to **singulate tree saplings**.

It uses:
- A **conveyor belt**, **gantry robot** and **elbow manipulator** (UR3e robot)
- A **YOLOv8**-trained vision system
- A **custom annotated dataset**
- Real-time decision making within a **ROS-based framework**

<p align="center">
<img width="600" src="https://github.com/user-attachments/assets/8705b03b-f8d9-41b4-afc6-a9b20bce04e3" />
</p>
<p align="center">Figure 1. Flowchart of the singulation algorithm.</p>

---

## 🤖 System Components

We use **ROS Noetic** to integrate:
- **RealSense camera**
- **UR3e robot**
- **Gantry robot**
- **Conveyor belt**
- A **Decision node** to coordinate all components

<p align="center">
<img src="https://github.com/user-attachments/assets/e20ad3ab-51f4-4e5f-8c75-d209c12f5b43" width="300"/>
</p>
<p align="center">Figure 2. System setup</p>

---

## 🚀 How to Launch

In Realsystem to bring up with the elbow manipulator, which is a UR3e robot, we need to run both the robot and MoveIt in the command line using the following commands:

##
    roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.100
##
    roslaunch ur3e_moveit_config moveit_planning_execution.launch limited:=true

The 'Camera' node detects individual trees and bunches, passing their position and orientation to an intermediate node that buffers the latest 10 frames. This buffering helps reduce the likelihood of missing a tree due to bounding box flickering caused by vision system uncertainty.

Launch the 'Camera' node:
##
    rosrun Camera camera.py
Launch the intermediate node:
##
    rosrun Connection connection.py

The gantry robot has three axes and is responsible for holding part of a tree bunch, allowing the UR3e robot to split the remaining portion. We have three nodes for three axes.

Launch the 'Gantry' nodes:
##
    rosrun Gantry gantryx.py
##
    rosrun Gantry gantryy.py
##
    rosrun Gantry gantryz.py    

The conveyor belt is responsible for transporting the bunches of trees toward the robots for singulation.

Launch the 'Conveyor' node:
##
    rosrun Conveyor conveyor.py

The 'Decision' node contains the algorithm that controls the singulation process. 

Launch the 'Decision' node:
##
    rosrun Decision decide.py
    
🔄 Closed-loop System Workflow
    
We use ROS Noetic to establish a closed-loop system integrating the RealSense camera, UR3e robot, Gantry robot, and conveyor belt. 

The system operates as follows:

https://github.com/user-attachments/assets/db30ec71-7a7f-44c0-b000-a3754b6d9e29
<p align="center" >
Figure4.Real closed-loop system.

The 'Decision' node:

-Instructs robots to singulate trees

-Commands the conveyor to stop/start based on object detection results

The flowchart for the decision algorithm is presented in Figure 5.
<p align="center" >
<img width="993" height="508" alt="ROS architecture" src="https://github.com/user-attachments/assets/a9f20d7e-82d4-4abe-96eb-d46144257692" />
<p align="center" >
Figure5. ROS architecture

🎯 'Camera' node & Object Detection

The 'Camera' node uses the trained model to detect single trees and bunches, reporting their position and orientation. The 'Decision' node then coordinates the robots for singulation or instructs the conveyor to stop or run.
<p align="center" >
<img src="https://github.com/user-attachments/assets/fda4ed9b-a95c-4a78-9810-d9bb0b86b5de"alt="Picture3" width="300" height="200" />
<p align="center" >
Figure6.Actual frames of the camera, with bounding boxes in red, and images of the robot in operation.

 🌳 Splitting Tree Bunches
 
The gantry robot holds part of the bunch, while the UR3e robot performs the splitting.

<p align="center" >
<img src="https://github.com/user-attachments/assets/592f1ade-24c8-4b41-9a09-9ef617e87176"alt="Picture4" width="300" height="200" />
    
<p align="center" >
Figure3.Splitting bunches of trees.

