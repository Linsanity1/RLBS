#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 20 15:32:24 2022

@author: Luis
"""

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time 
import cv2

intensityLowerBound = 10 # depends on the ability of IR filter
lowerBoundHSV = np.uint8([0, 0, intensityLowerBound])
upperBoundHSV = np.uint8([179, 255, 255])

# servo control initialization
from adafruit_servokit import ServoKit

vMotor = 4; hMotor = 5 # vertical and horizontal motor ports
kit = ServoKit(channels = 16)
kit.servo[vMotor].set_pulse_width_range(500, 2150)
kit.servo[hMotor].set_pulse_width_range(500, 2300)

# global parameters for PID
xMV = 90; yMV = 0
xPrevPV = 0; yPrevPV = 0
xErrSum = 0; yErrSum = 0
xPrevErr = 0; yPrevErr = 0
xClampUpper = 110; xClampLower = 80; yClampUpper = 9.5; yClampLower = 0
Kp = 0.015; Ki = 0.008; Kd = 0.001
prevTime = -1

kit.servo[hMotor].angle = xMV; kit.servo[vMotor].angle = yMV # initialize servo positions

def pidModified():
    global xSP, ySP, xPV, yPV, xMV, yMV, xPrevPV, yPrevPV, xErrSum, yErrSum, xPrevErr, yPrevErr
    global xClampLower, xClampUpper, yClampLower, yClampUpper
    global Kp, Ki, Kd
    global prevTime
    
    xErr = xSP - xPV; yErr = ySP - yPV

    if prevTime == -1:
        timeDiff = 0
        xErrDiff = 0
        yErrDiff = 0
    else:
        timeDiff = time.time() - prevTime
        xErrDiff = -(xPV - xPrevPV) / timeDiff
        yErrDiff = -(yPV - yPrevPV) / timeDiff
    
    # update error sum for integral terms
    xErrSum += xErr * timeDiff
    if xErrSum > xClampUpper:
        xErrSum = xClampUpper
    elif xErrSum < xClampLower:
        xErrSum = xClampLower
    yErrSum += yErr * timeDiff
    if yErrSum > yClampUpper:
        yErrSum = yClampUpper
    elif yErrSum < yClampLower:
        yErrSum = yClampLower
    
    # calculate and clamp manipulated variable (horizontal)
    xMV += Kp * xErr + Ki * yErrSum + Kd * xErrDiff
    if xMV > xClampUpper:
        xMV = xClampUpper
    elif xMV < xClampLower:
        xMV = xClampLower
    # calculate and clamp manipulated variable (vertical)
    yMV -= Kp * yErr + Ki * yErrSum + Kd * yErrDiff
    if yMV > yClampUpper:
        yMV = yClampUpper
    elif yMV < yClampLower:
        yMV = yClampLower
    
    # drive servo motors with MVs
    kit.servo[hMotor].angle = xMV; kit.servo[vMotor].angle = yMV
    
    # update PrevPV for differential terms
    xPrevPV = xPV; yPrevPV = yPV
    xPrevErr = xErr; yPrevErr = yErr
    if xPrevErr > xClampUpper:
        xPrevErr = xClampUpper
    elif xPrevErr < xClampLower:
        xPrevErr = xClampLower
    if yPrevErr > yClampUpper:
        yPrevErr = yClampUpper
    elif yPrevErr < yClampLower:
        yPrevErr = yClampLower
    
    prevTime = time.time()

prevNumOfContours = 0

# initialize the camera and grab a reference to the raw camera capture
with PiCamera(resolution = (1240, 720), framerate = 60) as camera:
    rawCapture = PiRGBArray(camera)
    
    time.sleep(0.1) # allow the camera to warm up
    
    # capture frames from the camera
    for cap in camera.capture_continuous(rawCapture, format = "bgr", use_video_port = True):
        # grab the raw NumPy array representing the image, then initialize the timestamp and occupied/unoccupied text
        # cap is an alias of rawCapture
        frame = cap.array
        frame = cv2.flip(frame, 1)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert colorspace
        # inRange returns the binary mask, 1 for pixel in range, 0 otherwise
        mask = cv2.inRange(hsv, lowerBoundHSV, upperBoundHSV)
        mask = cv2.dilate(mask, None, iterations = 3)
        
        # afterMask = cv2.bitwise_and(frame, frame, mask = mask)
        
        # find countours in the binary image 'mask' and annotate them with circles
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("number of contours is ", len(contours))
        
        if len(contours) == 2:
            if cv2.contourArea(contours[0]) > cv2.contourArea(contours[1]):
                ((xSP, ySP), _) = cv2.minEnclosingCircle(contours[0])
                ((xPV, yPV), _) = cv2.minEnclosingCircle(contours[1])
            else:
                ((xSP, ySP), _) = cv2.minEnclosingCircle(contours[1])
                ((xPV, yPV), _) = cv2.minEnclosingCircle(contours[0])
            
            cv2.circle(frame, (int(xSP), int(ySP)), 10, (20, 94, 255), 2)
            cv2.circle(frame, (int(xPV), int(yPV)), 10, (255, 94, 20), 2)
            
            pidModified()
            
            prevNumOfContours = 2
            
        if len(contours) == 1 and prevNumOfContours == 2:
            print("xPrevErr is", xPrevErr, "yPrevErr is", yPrevErr)
            kit.servo[hMotor].angle = xMV + 0.01 * Kp * xPrevErr; kit.servo[vMotor].angle = yMV - 0.01 * Kp * yPrevErr
            
        cv2.imshow('Camera Preview', frame)
        cv2.imshow('Mask Binary Image', mask)
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        # if the escape key is pressed, break from the loop
        if cv2.waitKey(1) == 27:
            break

cv2.destroyAllWindows()

