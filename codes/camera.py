#!/usr/bin/env python
import rospy
from camera.msg import pos
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

def pixel_coords_to_mask(pixel_coords, image_shape):
    print('INSIDE PIXEL_COORDS_TO_MASK: PIXEL_COORDS SIZE=', end='')
    print(np.shape(pixel_coords))
    print('IMAGE_SHAPE SIZE=', end='')
    print(np.shape(image_shape))
    mask = np.zeros(image_shape[:2], dtype=np.uint8)

    for coords in pixel_coords:
        if len(coords) == 0:  # Check for empty coords
            print("Invalid pixel coordinates encountered, skipping.")
            continue
            
        try:
            cv2.fillPoly(mask, [np.array(coords, dtype=np.int32)], 255)
        except cv2.error as e:
            print("An error occurred while filling the polygon:", e)
    return mask

def find_endpoints(mask_image):
    contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if contours:
        contour = contours[0]
        if len(contour) < 5:
            print("Not enough points to fit an ellipse.")
            return (999, 999), (999, 999), 9, 9, 9

        try:
            ellipse = cv2.fitEllipse(contour)
        except cv2.error as e:
            print("Ellipse fitting failed:", e)
            return (999, 999), (999, 999), 9, 9, 9

        center, (minor_axis, major_axis), angle = ellipse
        minor_len = min(minor_axis, major_axis) / 2

        # Check for invalid minor_len
        if not np.isfinite(minor_len) or minor_len <= 0:
            print("Invalid minor_len:", minor_len)
            return (999, 999), (999, 999), 9, 9, 9

        angle_rad = np.deg2rad(angle)
        cos_angle = np.cos(angle_rad)
        sin_angle = np.sin(angle_rad)

        intersections = []
        depths = []

        for t in np.linspace(-minor_len, minor_len, max(1, int(minor_len * 2))):
            x = int(center[0] + t * cos_angle)
            y = int(center[1] + t * sin_angle)
            if cv2.pointPolygonTest(contour, (x, y), False) >= 0:
                intersections.append((x, y))
                depth_value = depth_frame.get_distance(x, y)
                depths.append(depth_value)

        if intersections:
            average_depth = np.mean(depths) if depths else 0
            return intersections[0], intersections[-1], angle, minor_len, average_depth
        else:
            print("No intersections found.")
            return (999, 999), (999, 999), 9, 9, 9
    else:
        print("No contours found in the mask image.")
        return (999, 999), (999, 999), 9, 9, 9


# Initialize the YOLOv8 model
model = YOLO('/home/admin1/best.pt')
img_shape = (640, 480)
# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Configure the color stream
config.enable_device('238122070540') 
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream

# Start the pipeline
pipeline.start(config)

x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
z = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
o = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
d = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

a1, a2, b, c = 0, 0, 0, 0

xmin, xmax, ymin, ymax = -350, 400, 200, 495

# Initialize the ROS node
rospy.init_node('talker', anonymous=True)

pub = rospy.Publisher('pos_topic', pos, queue_size=10)
rate = rospy.Rate(1)  # Hz

try:
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_JET)

        # Run YOLOv8 inference on the color frame
        results = model(color_image, conf=0.05, iou=0.1)

        color_image = results[0].plot()

        for j in range(10):  # 10 is the array size
            x[j] = 0
            y[j] = 0
            z[j] = 0
            o[j] = 0
            d[j] = 0

        max_objects = 10
        if results[0].masks is not None:
            for result in results:
                masks = result.masks  # masks
                boxes = result.boxes  # boxes
                class_type = result.boxes.cls

                # Limit processing to the size of the arrays
                num_masks = min(len(masks.xy), max_objects)

                # Loop over each mask
                for mask_idx in range(num_masks):
                    mask_coords = masks.xy[mask_idx]
                    mask_img = pixel_coords_to_mask([mask_coords], img_shape)

                    endpoint1, endpoint2, orientation, minor_len, average_depth = find_endpoints(mask_img)
                    print("Orientation with respect to X-axis:", orientation)
                    print("End points along minor axis:", endpoint1, endpoint2)
                    print("Object ID:", mask_idx)
                    print("Class ID:", int(class_type[mask_idx]))
                    print("average_depth(meter):", average_depth)

                    # Draw the minor axis
                    cv2.line(color_image, endpoint1, endpoint2, (255, 0, 0), 2) 
                      
                    z[mask_idx] = int(class_type[mask_idx])
                    if z[mask_idx] == 0:
                      x[mask_idx] = (endpoint2[1]+endpoint1[1])/2
                    if z[mask_idx] == 1:                     
                      x[mask_idx] = endpoint2[1]
                    #x[mask_idx] = endpoint2[1]                    
                    o[mask_idx] = 90 - orientation
                    d[mask_idx] = average_depth
                    x[mask_idx] = (x[mask_idx] * 1.704) - 506 + 160 - 20   #x calibration for ur3 and gantry are the same
                    
                    if z[mask_idx] == 0:
                        y[mask_idx] = endpoint2[0]
                        y[mask_idx] = (y[mask_idx] * 1.68) + 100 + 10    # y calibration for ur3

                        if x[mask_idx] < xmin or x[mask_idx] > xmax or y[mask_idx] < ymin or y[mask_idx] > ymax or (x[mask_idx]) * (y[mask_idx]) > 120000 or (x[mask_idx]) * (y[mask_idx]) < -114000 or (x[mask_idx] < 70 and y[mask_idx] < 305):
                            
                            x[mask_idx], y[mask_idx], z[mask_idx], o[mask_idx], d[mask_idx] = 0, 0, 0, 0, 0


                    if z[mask_idx] == 1:
                        yb = endpoint2[0]  # y of bunch
                        yb = (yb * 1.68) + 100 + 10 
                        # print("yb = ")
                        # print(yb)
                        y[mask_idx] = (endpoint2[1] - endpoint1[1]) * 1.704  # width of bunch after calibration
                        # print("x[mask_idx]=")
                        # print(x[mask_idx])
                        # print("y[mask_idx]=")
                        # print(y[mask_idx])
                        if y[mask_idx] < 25 and y[mask_idx] != 0: #report it as single(if No contours found in the mask image(999) then y[mask_idx] = 0)
                           x[mask_idx] = x[mask_idx] - (y[mask_idx]/2) #report the center
                           y[mask_idx] = (endpoint2[0] * 1.68) + 100 + 10 

                           z[mask_idx] = 0
                           if x[mask_idx] < xmin or x[mask_idx] > xmax or y[mask_idx] < ymin or y[mask_idx] > ymax or (x[mask_idx]) * (y[mask_idx]) > 121000 or (x[mask_idx]) * (y[mask_idx]) < -114000 or (x[mask_idx] < 50 and y[mask_idx] < 305):
                              #print("make zero")
                              x[mask_idx], y[mask_idx], z[mask_idx], o[mask_idx], d[mask_idx] = 0, 0, 0, 0, 0                           

                        #if z[mask_idx] == 1 and (x[mask_idx] < -250 or x[mask_idx] > 100):  #-250 was -200
                        if z[mask_idx] == 1 and (x[mask_idx] < -250 or yb > 600 or y[mask_idx] == 0 or x[mask_idx] > 999):  #(if No contours found in the mask image(999) then y[mask_idx] = 0)
                            x[mask_idx], y[mask_idx], z[mask_idx], o[mask_idx], d[mask_idx] = 0, 0, 0, 0, 0

                            
        # Collect non-zero elements into a new list
        non_zero_elements = [(x[i], y[i], z[i], o[i], d[i]) for i in range(len(x)) if not (x[i] == 0 and y[i] == 0 and z[i] == 0 and o[i] == 0 and d[i] == 0)]

# Step 2: Initialize lists for tracking indices and values
        indices_when_z_is_0 = []
        values_when_z_is_0 = []
        x_when_z_is_1 = []
        differences_when_z_is_1 = []

# Step 3: Populate lists based on z values
        for j in range(len(z)):
            if z[j] == 0:
              indices_when_z_is_0.append(j)
              values_when_z_is_0.append(x[j])
            elif z[j] == 1:
              x_when_z_is_1.append(x[j])
              differences_when_z_is_1.append(x[j] - y[j])

# Step 4: Find indices to delete
        indices_to_delete = set()

        for index, value in zip(indices_when_z_is_0, values_when_z_is_0):
          for x_val, diff_val in zip(x_when_z_is_1, differences_when_z_is_1):
            if diff_val-50 <= value <= x_val+50:
               indices_to_delete.add(index)

# Step 5: Delete marked indices in reverse order
        for i in sorted(indices_to_delete, reverse=True):
          del x[i]
          del y[i]
          del z[i]
          del o[i]
          del d[i]

# Step 6: Collect non-zero elements again after deletions
        non_zero_elements = [(x[i], y[i], z[i], o[i], d[i]) for i in range(len(x)) if not (x[i] == 0 and y[i] == 0 and z[i] == 0 and o[i] == 0 and d[i] == 0)]

        while len(x) < 10:
            x.append(0)
        while len(y) < 10:
            y.append(0)
        while len(z) < 10:
            z.append(0)
        while len(o) < 10:
            o.append(0)
        while len(d) < 10:
            d.append(0)

# Step 7: Fill the arrays with non-zero elements followed by zeros
        for j in range(10):
          if j < len(non_zero_elements):
            x[j], y[j], z[j], o[j], d[j] = non_zero_elements[j]
          else:
            x[j], y[j], z[j], o[j], d[j] = 0, 0, 0, 0, 0
 
# Ensure the lists are exactly of length 10
        x = x[:10]
        y = y[:10]
        z = z[:10]
        o = o[:10]
        d = d[:10]



        # Collect non-zero elements into a new list
        non_zero_elements = [(x[i], y[i], z[i], o[i], d[i]) for i in range(10) if not (x[i] == 0 and y[i] == 0 and z[i] == 0 and o[i] == 0 and d[i] == 0)]

        # Fill the arrays with non-zero elements followed by zeros
        for j in range(10):
            if j < len(non_zero_elements):
                x[j], y[j], z[j], o[j], d[j] = non_zero_elements[j]
            else:
                x[j], y[j], z[j], o[j], d[j] = 0, 0, 0, 0, 0


        pos_msg = pos()
        for i, value in enumerate(x):
            setattr(pos_msg, f'x{i + 1}', value)

        for i, value in enumerate(y):
            setattr(pos_msg, f'y{i + 1}', value)

        for i, value in enumerate(z):
            setattr(pos_msg, f'z{i + 1}', value)

        for i, value in enumerate(o):
            setattr(pos_msg, f'o{i + 1}', value)

        for i, value in enumerate(d):
            setattr(pos_msg, f'd{i + 1}', value)           

        pub.publish(pos_msg)

        print(pos_msg.x1, pos_msg.y1, pos_msg.d1, pos_msg.z1, pos_msg.o1)
        print(pos_msg.x2, pos_msg.y2, pos_msg.d2, pos_msg.z2, pos_msg.o2)
        print(pos_msg.x3, pos_msg.y3, pos_msg.d3, pos_msg.z3, pos_msg.o3)
        print(pos_msg.x4, pos_msg.y4, pos_msg.d4, pos_msg.z4, pos_msg.o4)

        h, w, _ = color_image.shape  # Get the height and width of the color image

        # Define calibration (mapping) between your custom x-coordinates and image coordinates
        # For this example, assume the middle of the image corresponds to x = 0 in your coordinate system
        image_center_x = w // 2
        image_center_y = h // 2


        # Create a transparent overlay (same size as the color image)
        overlay = np.zeros((h, w, 3), dtype=color_image.dtype)

        origin_x = 0
        origin_y = h // 2  # Middle of the image vertically
        

        # Apply the overlay for x > 0 and y > 0 condition (top-right from the middle-left)
        for b in range(h):
            for a in range(w):
        # Convert pixel coordinates to custom x and y values
                custom_x = a - origin_x  # Custom x starts from the middle-left
                custom_y = origin_y - b  # Custom y starts from the middle (positive upwards)
                customc_y=custom_x
                customc_x=custom_y
                if customc_y < 62 or customc_y > 237 or customc_x < -208 or customc_x > 232 or (customc_x) * (customc_y) > 35437 or (customc_x) * (customc_y) < -26730 or (customc_x>-2 and customc_y<120):
                    overlay[b, a] = [0, 255, 255] 

      

        # Set transparency level (alpha value)
        alpha = 0.5  # Adjust the alpha for more/less transparency

        # Blend the overlay with the original image using the transparency level
        cv2.addWeighted(overlay, alpha, color_image, 1 - alpha, 0, color_image)

        #Display the color frame with ellipsoid fitting results
        cv2.imshow('Intel Realsense', color_image)

        key = cv2.waitKey(1)

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

