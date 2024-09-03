import PySimpleGUI as sg
#import csv
from os.path import exists
import numpy as np
from re import search
import json
from datetime import date,datetime
#import re
#import itertools
import ast
import os
import cv2
import sys
import pyrealsense2.pyrealsense2 as rs
from helper import saveDataThread
from time import sleep

def read_config():
    ids = np.genfromtxt('Config/layout_id.csv', delimiter = ',', dtype = None, encoding=None)
    monitoring = np.genfromtxt('Config/monitor_status.csv', dtype = int, delimiter = ',')
    stall_sow_id = np.genfromtxt('Config/sow_stall_id.csv', dtype = int, delimiter = ',')

    if len(ids.shape)==1:
        ids = np.transpose(ids.reshape(-1,1))
        monitoring = np.transpose(monitoring.reshape(-1,1))
        stall_sow_id = np.transpose(stall_sow_id.reshape(-1,1))

    mask = stall_sow_id>0
    stall_sow_id = stall_sow_id.astype(str)
    stall_sow_id[mask] = (np.char.zfill(stall_sow_id[mask], 5))
    return ids, monitoring, stall_sow_id

def stream_save(monitor, ids,pig_id, i):
    #stream_save(ids[,i],stall_sow_id[,i], i)
    print(monitor)
    print(ids)
    print(pig)
    
    if int(pig_id) == 0:
        path ="/home/pi/Desktop/ROBOTSOFTWARE/Data/" +"STALL_"+ ids + "/"#+imgname
        filename = ids
    else:
        path ="/home/pi/Desktop/ROBOTSOFTWARE/Data/" +"PIG"+ pig_id + "/"#+imgname
        filename = "PID_"+str(pig_id)
    if not os.path.exists(path):
            os.makedirs(path)

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
    config.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)
    config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)

    config.enable_device(camera_idj)
    profile = pipeline.start(config)
    get_intrinsic = profile.get_stream(rs.stream.depth)
    intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    clipping_distance_in_meters = 1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale
    align_to = rs.stream.depth
    align = rs.align(align_to)
    finish_capturing = False
    f_count = 0
    try:
        while not finish_capturing:
            f_count = f_count+1
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            if not depth_frame:
                continue
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            cv2.namedWindow(ids, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(ids, depth_colormap)
            cv2.waitKey(1)
            if cv2.getWindowProperty(ids,1) == -1.0:#<0:
                print("stop autopilot")
                os._exit(0)
                #quit()
                #sys.exit()
            #cv2.waitKey(1)

            if f_count > 45:
                #print("Saving data")
                t=datetime.now()
                thread1=saveDataThread(2*i+j, aligned_frames,intr, path, filename,t)
                thread1.start()
                finish_capturing = True
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        
ids, monitoring, stall_sow_id = read_config()
c,r = ids.shape
###Configure Sensor
ctx = rs.context()
#print(ctx)
camera_id = []
for d in ctx.devices:
    print(d.get_info(rs.camera_info.serial_number))
    camera_id.append(d.get_info(rs.camera_info.serial_number))
    

pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device(camera_id[0])
config_1.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
config_1.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
config_1.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)


pipeline_2=rs.pipeline()
config_2 = rs.config()
config_2.enable_device(camera_id[0])
config_2.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
config_2.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
config_2.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)

##############MOTOR SETUP
#testStepper = Stepper([13,15,11 ,18,40,38])#140



while True:
    t1 = datetime.now()
    totalStep=0
    #print("wait till 5 min")
    if t1.minute % 1== 0:
        print("Leaving dock")
        #stepper motor leave dock
        for i in range(r):
            #move to next
            print("Move to next stop")
            
            if moitoring[:,i] !=0:
                try:
                    stream_save(moitoring[:,i],ids[:,i],stall_sow_id[:,i], i)
                        
                except Exception as e:
                            #fail_attemp =fail_attemp+1
                            #print(ids[j,i], "Trying again", fail_attemp)
                    print(e)
                    e1=0
            else:
                print("Skip line")
                
            if i == max(range(r)):
                print("return to dock")
            else:
                print("move forward")
                    









