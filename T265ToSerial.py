#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
sys.path.append("/usr/local/lib/")
# Import the libraries
#import pyrealsense2 as rs
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import serial


H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None

H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)


body_offset_x = 10 #cm
body_offset_y = 0  
body_offset_z = 0  
scale_factor = 100.0
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)
ser = serial.Serial('/dev/ttyTHS1', baudrate = 115200,bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
try:
    #
    while (True):
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            data = pose.get_pose_data()
            H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]) 
            H_T265Ref_T265body[0][3] = data.translation.x * scale_factor
            H_T265Ref_T265body[1][3] = data.translation.y * scale_factor
            H_T265Ref_T265body[2][3] = data.translation.z * scale_factor
            # Transform to aeronautic coordinates (body AND reference frame!)
            H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody))
            H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
            H_body_camera[0][3] = body_offset_x
            H_body_camera[1][3] = body_offset_y
            H_body_camera[2][3] = body_offset_z
            H_camera_body = np.linalg.inv(H_body_camera)
            H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))
            Roll, Pitch, Yaw = np.array(tf.euler_from_matrix( H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi
            posX, posY, posZ = np.array(tf.translation_from_matrix(H_aeroRef_aeroBody))

            if (Roll < 0):
                Roll = Roll + 360

            if (Pitch < 0):
                Pitch = Pitch + 360

            if (Yaw < 0):
                Yaw = Yaw + 360

            string_message="a{:04d}{:04d}{:04d}{:04d}{:04d}{:04d}".format(int(Roll), int(Pitch), int(Yaw),
                                                                         (5000 + int(posX)), (5000 + int(posY)),  (5000 + int(posZ)))
            ser.write(string_message.encode('utf-8'))
            print (string_message)
            time.sleep(.01)

        
finally:
    pipe.stop();
    ser.close()
