# ISILDUR
This project is about developing a closed-loop system that integrates robotic manipulators with a deep learning-based visual object recognition and pose estimation system for singulating tree saplings. Robotic manipulators, including a gantry robot and an elbow manipulator, emulate human hand movements. A custom dataset of annotated images was created and augmented to train the YOLOv8 model for localizing individual trees and tree bunches. The system operates in real-time within a ROS-based closed-loop framework, ensuring seamless interaction between visual feedback and robotic actions.

To work with the elbow manipulator which is a UR3 robot we need to run the robot as well as Moveit in the command line through these codes:
'''roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.100'''
'''roslaunch ur3e_moveit_config moveit_planning_execution.launch limited:=true'''
