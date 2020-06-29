#!/usr/bin/python
# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
cfg = pipeline.start(config)
#dev = cfg.get_device()
#depth_sensor = dev.first_depth_sensor()
#depth_sensor.set_option(rs.option.visual_preset, 4)

iteration = 0;
preset = 0;
preset_name = '';

cap = cv2.VideoCapture(1)

try:
    while(True):

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        #iteration = iteration + 1
        #if iteration > 100:
            #preset = preset + 1
            #iteration = 0
            #range = depth_sensor.get_option_range(rs.option.visual_preset)
            #preset = preset % range.max
            #depth_sensor.set_option(rs.option.visual_preset, preset)
            #preset_name = depth_sensor.get_option_value_description(rs.option.visual_preset, preset)
        
        # Convert images to numpy arrays
        #depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, None, 0.5, 0), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))
        images = color_image

        #font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(images, preset_name,(720,1300), font, 4,(255,255,255),2,cv2.LINE_AA)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        ret, RealSense = cap.read()

        cv2.imshow('RealSense', images)
        #cv2.imshow('Real', images)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('t'):
            cv2.imwrite('photo.png',images)
        #result = False
        
        
        #cv2.imshow('frame', frame)
        elif key & 0xFF == ord('q')or key ==27:
            break
        
      
finally:
    cap.release()
    cv2.destroyAllWindows()
    # Stop streaming
    pipeline.stop()
