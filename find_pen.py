# First import the library
from numpy.typing import _128Bit
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

import modern_robotics as mr

X_old = 0
Y_old = 0
D_old = 0

# Import robot file and set pose to home
import move_robot
move_robot.sleep_pose()
move_robot.move_wrist()
move_robot.open_grip()

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
cfg = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = cfg.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 0.5 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #images = np.hstack((bg_removed, depth_colormap))

        #cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        #cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        # Convert BGR to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # define range of purple color in HSV
        lower_purple = np.array([110,100,20])
        upper_purple = np.array([140,255,255])
        # Threshold the HSV image to get only purple colors
        mask = cv2.inRange(hsv, lower_purple, upper_purple)
        # Bitwise-AND mask and original image
        #res = cv2.bitwise_and(color_image,color_image, mask = mask)
                        
        # Filter image using erosion technique
        # Take matrix of size 5 as the kernel (matrix used to convolve image)
        kernel = np.ones((5,5), np.uint8)
        mask_eroded = cv2.erode(mask, kernel, iterations=2)
        mask_filtered = cv2.dilate(mask_eroded, kernel, iterations=2)

        # Find contours
        contours, hierarchy = cv2.findContours(mask_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contour_areas = []

        # Find centroid of first contour
        if len(contours) > 0:
            for x in contours:
                contour_areas.append(cv2.contourArea(x))
            big_contour_index = np.argmax(contour_areas)

            # Find centroid of largest contour
            M = cv2.moments(contours[big_contour_index])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(mask_filtered, (cx, cy), 7, (150, 150, 0), -1)
                #print(f"Centroid of Largest Contour: {cx},{cy}")

                #Find centroid depth
                if cx < 640 and cy < 480:
                    centroid_depth_image = depth_image[cy][cx]
                    centroid_depth = centroid_depth_image*depth_scale
                    #print(f"Centroid depth: {centroid_depth}") 
                else:
                    centroid_depth = 0.3
                    #print(f"Centroid depth: {centroid_depth}") 

            # Note in the example code, cfg is misleadingly called "profile" but cfg is a better name
            profile = cfg.get_stream(rs.stream.color)
            intr = profile.as_video_stream_profile().get_intrinsics()
            pen_coordinates = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], centroid_depth)
            
            X = pen_coordinates[0]
            Y = pen_coordinates[1]
            D = pen_coordinates[2] 

            # Filter out extremes
            if np.absolute(X - X_old) < 0.05: 
                pass
            else:
                X = X_old
                Y = Y_old
                D = D_old

            print(f"Pen Location     X: {round(X,3)}   Y:{round(Y,3)}   D: {round(D,3)}")


            # Set robot waist position
            move_robot.move_waist(X, Y, D)
            
            # Move robot forward
            status = move_robot.move_forward(X, Y, D)

            # Grab pen
            if status:
                move_robot.grab_pen()
                break
            else:
                pass

            X_old = X
            Y_old = Y
            D_old = D




        else: #If no contours, don't do anything
            pass
            
        #Draw contours
        cv2.drawContours(mask_filtered, contours, -1, (0,255,0), 1)
        
        # Show frames
        cv2.imshow('Frame',color_image)
        #cv2.imshow('Mask',mask)
        cv2.imshow('Mask Filtered', mask_filtered)

finally:
    pipeline.stop()
