#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 15:04:53 2023

@author: Luis
"""

import cv2
import numpy as np
import subprocess as sp
import time
import datetime
########################################################################################
#
# Initialization stage
#

# initialize seiral with a default baudrate of 115200
import serial
serial_port = serial.Serial("/dev/serial0", 115200)

# initialize stepper motors
from Utils.StepperMotor import Motor
motor_horizontal = Motor(serial_port, b'\x01')
motor_horizontal.set_angle_limits(-10, 10) # motor1 is allowed to move within -2 to 2 deg
motor_vertical = Motor(serial_port, b'\x02')
motor_vertical.set_angle_limits(-10, 10)

# initialize PID controllers
from Utils.PID import PID
kp = -0.01
ki = -0.1
kd = 0
pid_vertical = PID(Kp = kp, Ki = ki, Kd = kd, setpoint = 0, sample_time = 0.033, output_limits = (-20, 20), differential_on_measurement = True)
pid_horizontal = PID(Kp = kp, Ki = ki, Kd = kd, setpoint = 0, sample_time = 0.033, output_limits = (-20, 20), differential_on_measurement = True)

# initialize a ReceivedPoints and a tracker to manage camera-captured contours
from Utils.ReceivedPoints import ReceivedPoints
from Utils.Tracker import Tracker
received_points = ReceivedPoints()
tracker = Tracker(received_points)

# global parameters
prevNumOfContours = 0
frame_count = 0
motor_enable = False

show_image_enable = False
write_enable = True
debug_enable = False
########################################################################################


frames = []
frame_count = 0

# Video capture parameters
# (1280, 960) @ 40 fps
(width, height) = (1280, 960)
bytesPerFrame = width * height
fps = 40

# "raspividyuv" is the command that provides camera frames in YUV format
#  "--output -" specifies stdout as the output
#  "--timeout 0" specifies continuous video
#  "--luma" discards chroma channels, only luminance is sent through the pipeline
# see "raspividyuv --help" for more information on the parameters
videoCmd = "raspividyuv -w " + str(width) + " -h " + str(height) + " --output - --timeout 0 --framerate " + str(fps) + " --luma --nopreview"
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string

with sp.Popen(videoCmd, stdout = sp.PIPE) as cameraProcess:
    start_time = time.time()
    
    # wait for the first frame and discard it (only done to measure time more accurately)
    rawStream = cameraProcess.stdout.read(bytesPerFrame)
    
    print("Starting test protocol.")
    tracker.reset_storage()
    
    while frame_count < 1500:
        cameraProcess.stdout.flush() # discard any frames that we were not able to process in time
        
        frame = np.frombuffer(cameraProcess.stdout.read(bytesPerFrame), dtype=np.uint8) # Parse the raw stream into a numpy array
        
        if frame.size != bytesPerFrame:
            print("Error: Camera stream closed unexpectedly")
            break
        
        frame.shape = (height, width) # set the correct dimensions for the numpy array
        
        frame = cv2.inRange(frame, 50, 255) # create binary image
        # frame = cv2.dilate(frame, None, iterations = 1)
        
        
        contours, _ = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("number of contours is ", len(contours))
        
        frame_count += 1
        frames.append(frame) # store frames for future visualization
        
        # control logics
        if motor_enable == False:
            if len(contours) == 5:
                (center1, _) = cv2.minEnclosingCircle(contours[0])
                (center2, _) = cv2.minEnclosingCircle(contours[1])
                (center3, _) = cv2.minEnclosingCircle(contours[2])
                (center4, _) = cv2.minEnclosingCircle(contours[3])
                (center5, _) = cv2.minEnclosingCircle(contours[4])
                point_set = (center1, center2, center3, center4, center5)
                
                tracker.update(point_set)
            
            if show_image_enable:
                cv2.imshow('Camera Preview', frame)
                if cv2.waitKey(1) == 27: # escape key pressed
                    break
                elif cv2.waitKey(1) == 32: # space key pressed
                    enable_motor = True
                    pid_horizontal.reset() # reset PID controllers, particularly the internal recorded time
                    pid_vertical.reset()
            elif frame_count >= 100:
                motor_enable = True
                pid_horizontal.reset() # reset PID controllers, particularly the internal recorded time
                pid_vertical.reset()
            
            continue
       
        
        if len(contours) == 5:
            (center1, _) = cv2.minEnclosingCircle(contours[0])
            (center2, _) = cv2.minEnclosingCircle(contours[1])
            (center3, _) = cv2.minEnclosingCircle(contours[2])
            (center4, _) = cv2.minEnclosingCircle(contours[3])
            (center5, _) = cv2.minEnclosingCircle(contours[4])
            
            point_set = (center1, center2, center3, center4, center5)
            tracker.update(point_set)
            (xSP, ySP), (xPV, yPV) = tracker.positions()
            
            pid_horizontal.setpoint = xSP
            xMV = pid_horizontal(xPV)
            time.sleep(0.005)
            motor_horizontal.move_to(xMV)
            
            if debug_enable:
                print("Integral term is", pid_horizontal._integral)
                print("xErr is", xSP - xPV)
                print("xMV (angle) is", xMV)
            
            pid_vertical.setpoint = ySP
            yMV = pid_vertical(yPV)
            time.sleep(0.005)
            motor_vertical.move_to(yMV)
            # print("yErr is", ySP - yPV)
            # print("yMV (angle) is", yMV)
            
            prevNumOfContours = 5
            xSP_prev = xSP
            xPV_prev = xPV
            ySP_prev = ySP
            yPV_prev = yPV
            
        if len(contours) == 4 and prevNumOfContours == 5: # and laser == on
            # add logic later.
            pid_horizontal.setpoint = xSP_prev
            xMV = pid_horizontal(xPV_prev)
            time.sleep(0.005)
            motor_horizontal.move_to(xMV)
            if debug_enable:
                print("xErr is", xSP_prev - xPV_prev)
                print("xMV (angle) is", xMV)
            
            pid_vertical.setpoint = ySP_prev
            yMV = pid_vertical(yPV_prev)
            time.sleep(0.005)
            motor_vertical.move_to(yMV)
            if debug_enable:
                print("yErr is", ySP_prev - yPV_prev)
                print("yMV (angle) is", yMV)
            
            prevNumOfContours = 4
            
        if show_image_enable:
            cv2.imshow('Camera Preview', frame)
            if cv2.waitKey(1) == 27: # escape key pressed
                break

    end_time = time.time()
    print("Done! Result: " + str(frame_count / (end_time - start_time)) + " fps")

time.sleep(0.1)
motor_vertical.move_to_origin()
time.sleep(0.1)
motor_horizontal.move_to_origin()
time.sleep(0.1)
cv2.destroyAllWindows()
serial_port.close()
tracker.plot_dist_func()


# print result to file
if write_enable:
    with open("Results/continuous_movement_1cm_s " + datetime.datetime.now().strftime("%b %d %H:%M:%S"), 'w') as file:
        time_string, dist_string = tracker.retrieve_stored_data_as_strings() 
        file.write(time_string)
        file.write('\n')
        file.write(dist_string)

if False:
    print("Display frames with OpenCV...")
    for frame in frames:
        cv2.imshow("frame", frame)
        cv2.waitKey(1)                                                                                                                                            

cv2.destroyAllWindows()

if False:
    print("Wrting frames to disk...")
    out = cv2.VideoWriter("Videos/continuous_movement_1cm_s_new.avi", cv2.VideoWriter_fourcc(*"MJPG"), frame_count / (end_time - start_time), (width, height))
    for frame in frames:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        out.write(frame_rgb)
    out.release()
    print("Finished recording video.")
