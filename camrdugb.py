#!/usr/bin/env python
import rospy
from camera.msg import pos
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import time

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Configure the color stream
config.enable_device('238122070540') 
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream

# Start the pipeline
pipeline.start(config)

# Load YOLO model
#model = YOLO('yolov8x.pt')  # Load a pretrained YOLO model
model = YOLO('bestb2.pt')

ii = [0,0,0,0,0,0,0,0,0,0]
x = [0,0,0,0,0,0,0,0,0,0]
y = [0,0,0,0,0,0,0,0,0,0]
z = [0,0,0,0,0,0,0,0,0,0]
k = 1
jj = [0,0,0,0,0,0,0,0,0,0] #rootb
b = 1
x1min,y1min,x2min,y2min = 0,0,0,0 

# Initialize the ROS node
rospy.init_node('talker', anonymous=True)

pub = rospy.Publisher('pos_topic', pos, queue_size=10)
rate = rospy.Rate(1)  # Hz

while not rospy.is_shutdown():
    # Wait for the next set of frames from the RealSense camera
    frames = pipeline.wait_for_frames()

    # Get the color and depth frames
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    if not color_frame or not depth_frame:
        continue

    # Convert the color frame to a numpy array
    color_image = np.asanyarray(color_frame.get_data())

    # Perform object detection with YOLO
    detections = model(color_image)

    # Display the frame with YOLO object detection
    cv2.imshow('RealSense Camera', detections[0].plot(1))
    cv2.waitKey(1)

    ss = detections[0]
    count = 0
    cb = 0
    for i in range(ss.boxes.shape[0]):        
        if ss.boxes[i].cls.cpu().numpy()[0] == 0:   #bestb2.pt-root
            ii[count] = i
            count = count +1
            k = 0  
            
        if ss.boxes[i].cls.cpu().numpy()[0] == 1:   #bestb2.pt-rootb    
            jj[cb] = i
            cb = cb +1
            b = 0
                      
    if k==0 or b==0:    
     for j in range(10): # 10 is the array size
       x[j] = 0
       y[j] = 0
       z[j] = 0
       
     if k == 0:
         
      for j in range(count):
        
        try:
            x1, y1, x2, y2 = ss.boxes[ii[j]].xyxy.cpu().numpy()[0]
            xm = (x1 + x2) / 2
            ym = (y1 + y2) / 2            
            depth = depth_frame.get_distance(int(xm), int(ym))  # Get depth at the object's position
            
            y[j] = xm 
            x[j] = ym
            z[j] = 0 #it is root
            
            k = 1
            
        except:
            # Handle the case when ss.boxes is empty
            print("Error processing object detection.")
            k = 1   
            
            
     if b == 0:    
 
       if cb == 1:
        try:                 
         x1, y1, x2, y2 = ss.boxes[jj[0]].xyxy.cpu().numpy()[0]          

         xm = (x1 + x2) / 2
         ym = (y1 + y2) / 2            
         depth = depth_frame.get_distance(int(xm), int(ym))  # Get depth at the object's position
            
         #y[count] = xm 
         y[count] = y2   #I need this for gantry robot and we donot need y coordination for now
         x[count] = y1
         z[count] = 1 #it is rootb
            
         b = 1
            
        except:
            # Handle the case when ss.boxes is empty
            print("Error processing object detection.")
            b = 1          

       else:
        for j in range(cb):
         try:                 
          x1, y1, x2, y2 = ss.boxes[jj[j]].xyxy.cpu().numpy()[0]  
        
          if j == 0:
            y1min = y1
            x1min = x1
            x2min = x2
            y2min = y2 
            
          if y1<y1min:
            x1min = x1, y1min = y1, x2min = x2, y2min = y2      
           
         except:
            # Handle the case when ss.boxes is empty
            print("Error processing object detection.")
            b = 1          
        xm = (x1min + x2min) / 2
        ym = (y1min + y2min) / 2            
        depth = depth_frame.get_distance(int(xm), int(ym))  # Get depth at the object's position
            
        #y[count] = xm 
        y[count] = y2min   #I need this for gantry robot and we donot need y coordination for now
        x[count] = y1min
        z[count] = 1 #it is rootb
            
        b = 1
       
       
     pos_msg = pos() 
     
     for i, value in enumerate(x):
        setattr(pos_msg, f'x{i + 1}', value)
        
     for i, value in enumerate(y):
        setattr(pos_msg, f'y{i + 1}', value)        
     
     for i, value in enumerate(z):
        setattr(pos_msg, f'z{i + 1}', value)
              
     pub.publish(pos_msg) 
     pos_msg.x1=(pos_msg.x1*-1.58)+222   
     print(pos_msg.x1,pos_msg.y1,pos_msg.x2,pos_msg.y2,pos_msg.x3,pos_msg.y3,pos_msg.x4,pos_msg.y4)
     #rate.sleep()            
    else:
        print("No object detected")

# Release the camera and close all OpenCV windows
pipeline.stop()
cv2.destroyAllWindows()

