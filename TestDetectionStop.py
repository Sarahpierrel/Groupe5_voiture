#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 19 17:01:06 2024

@author: louisepoteau
"""
from ultralytics import YOLO
import cv2

"""
We trained the model with YOLOV8 before and we just used the results now
model = YOLO('best.pt') 

Load video
VideoPath = './test.mp4'

"""



def detection(VideoPath, Model):


    cap = cv2.VideoCapture(VideoPath)
    
    Switch = True 
    # read frames
    
    while Switch:
        Switch, Frame = cap.read()
        if Switch:
    
            #Track objects
            #Model.track is a YOLOV8 module
            results = Model.track(Frame, persist=True)
            
            #Plot results
            AnnotedFrame = results[0].plot()
    
            #Visualize
            cv2.imshow('frame', AnnotedFrame)
            
            #If q is taped, the window will shut down
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

detection('./test4.mp4', YOLO('best.pt'))
