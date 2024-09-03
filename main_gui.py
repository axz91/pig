from OperationSetting_GUI import operation_setting_gui
import PySimpleGUI as sg
#import csv
from os.path import exists
import numpy as np
from Setup_Farm_Layout_GUI import setup_farm_layout_gui
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

def main_robot_gui_window(ids, monitoring, stall_sow_id):
    c,r = ids.shape
    stall_summary = [[sg.Frame("Monitoring status", 
        [
            #[sg.OptionMenu(values = sow_lists, default_value = ids_s[i][j],size = (int(w/(10*c*4)),3)) for i in range(c)] for j in range(r)
            [sg.Text(ids[i][j] +": "+ str(stall_sow_id[i][j]), size = (12,1), key="stall_sows"+str(i)+str(j), background_color = ["lightgray", "green"][monitoring[i,j]]) for i in range(c)] for j in range(r)
        ])
        ], [sg.Button("Edit",size = (12,1))]
    ]
    
    robot_image_control = [[sg.Frame("Robot Control",[
                                     [sg.Text("Manual Control"),  sg.Button("Stream_L"),sg.Button("Stream_R"),sg.Button("STOP STREAM"), sg.Button("Capture")],
                                     #[sg.Text("Image Viewer")],
                                     [sg.Image(filename="", key='image', size = (300,300)), sg.Text(key='-info-')],
                                     [sg.Text("Auto Pilot"), sg.Button("Start"), sg.Button("Pause"), sg.Button("Resume"), sg.Button("End")]
                                     ]
                                     )
                            ]]
    
    robot_window_layout = [[
            [sg.Column(stall_summary, expand_y = True,size=(250,80)),
            sg.VSeparator(),
             sg.Column(robot_image_control, expand_y = True, size = (950,80))
             ],
            [sg.HSeparator(pad =(5,5))]
         ]]
    window = sg.Window(title="SOW ROBOT", layout=robot_window_layout, resizable=True, no_titlebar=False, size=(1000,650), auto_size_text = True, element_justification='c',finalize=True)
    return window

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
#print(stall_sow_id)
def stream_save(camera_idj, ids, i,j):
    
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
                return False 
                #quit()
                #sys.exit()
            #cv2.waitKey(1)

            if f_count > 45:
                #print("Saving data")
                thread1=saveDataThread(2*i+j, aligned_frames,1,intr, "/home/pi/Desktop/ROBOTSOFTWARE/Capture", str(ids))
                thread1.start()
                finish_capturing = True
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()

        return True
        #print("close window")
            #cv2.waitKey(1)
                       

    
    
    
setting_exists = exists("Config/farm_setting.txt") or exists("Config/layout_id.csv")
if not setting_exists:
    setup_farm_layout_gui()

ids, monitoring, stall_sow_id = read_config()
if sum(monitoring.flatten())==0:
    #print(sum(monitoring.flatten())==0)
    operation_setting_gui(ids, monitoring, stall_sow_id)
    
ids, monitoring, stall_sow_id = read_config()
c,r = ids.shape
main_robot_windows = main_robot_gui_window(ids, monitoring,stall_sow_id)

stream = False
autopilot = False
###Configure Sensor
ctx = rs.context()
#print(ctx)
camera_id = []
for d in ctx.devices:
    print(d.get_info(rs.camera_info.serial_number))
    camera_id.append(d.get_info(rs.camera_info.serial_number))

main_robot_windows["Capture"].update(disabled = True)
empty_image = np.zeros((250,250))
emp_imgbytes = cv2.imencode('.png', empty_image)[1].tobytes()
main_robot_windows['image'].update(data=emp_imgbytes)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
config.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)

while True:
    event, values = main_robot_windows.read(timeout=20)
    #print(event)
    if len(camera_id)==1:
        main_robot_windows["Stream_R"].update(disabled = True)
    

    if event == sg.WIN_CLOSED or event == "Exit":
        main_window.close()
        break
    if event == "Edit":
        operation_setting_gui(ids, monitoring, stall_sow_id)
        ids, monitoring, stall_sow_id = read_config()
        for i in range(ids.shape[0]):
            for j in range(ids.shape[1]):
                main_robot_windows["stall_sows"+str(i)+str(j)].update(background_color=["lightgray", "green"][int(monitoring[i,j])])
                
    #if event == "Stream":
            
    if search("Stream_", event):
        # Start streaming
        pipeline = rs.pipeline()
        config = rs.config()
        if event == "Stream_L":
            config.enable_device(camera_id[0])
            main_robot_windows["Stream_L"].update(disabled = True)
            main_robot_windows["Stream_R"].update(disabled = False)

        elif event =="Stream_R":
            config.enable_device(camera_id[1])
            main_robot_windows["Stream_R"].update(disabled = True)
            main_robot_windows["Stream_L"].update(disabled = False)
       
        try:
            pipeline.stop()
        except:
            e=0
            #print("")
        profile = pipeline.start(config)
        get_intrinsic = profile.get_stream(rs.stream.depth)
        intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        temp_filter = rs.temporal_filter(1,1,3)
        clipping_distance_in_meters = 1 #1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
        align_to = rs.stream.depth
        align = rs.align(align_to)

        stream = True
        main_robot_windows["Capture"].update(disabled = False)

        
    if event == "STOP STREAM":
        stream = False
        main_robot_windows["Capture"].update(disabled = True)
        try:
            pipeline.stop()
            
        except:
            e=1
        try:
            cv2.destroyAllWindows()
        except:
            e=1
        main_robot_windows["Stream_L"].update(disabled = False)
        main_robot_windows["Stream_R"].update(disabled = False)
        main_robot_windows['image'].update(data=emp_imgbytes)

    if event =="Capture":
        stream = False
        folder_path = sg.popup_get_folder("select folder for destination")
        if folder_path != None:
            file_name = sg.popup_get_text("Enter File Name (Will save depth image, RGB image, and IR image)")
        if file_name != None:
            try:
                thread1=saveDataThread(1, aligned_frames,2,intr,folder_path, file_name)
                thread1.start()
            except:
                e=1
                #print("Failed to save image")
        else:
            sg.popup_ok("Didnt enter file name")
        try:
            pipeline.stop()
        except:
            print("")
    if event == "Start":
        main_robot_windows["Stream_L"].update(disabled = True)
        main_robot_windows["Stream_R"].update(disabled = True)

        autopilot = True            
    if event == "End":
        autopilot = False
        print("END")
    if stream:
        
        frames = pipeline.wait_for_frames(5000)
        aligned_frames = align.process(frames)
        # Get aligned frames
        #aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue
        #filtered = temp_filter.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        #print(depth_image.shape)
        depth_image = depth_image[:, 80:560]
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap = cv2.resize(depth_colormap, dsize = (250,250), interpolation = cv2.INTER_CUBIC)
        imgbytes = cv2.imencode('.png', depth_colormap)[1].tobytes()
        main_robot_windows['image'].update(data=imgbytes)
        

    while autopilot:
        print("leave dock")
        for i in range(r):
            for j in range(c):
                print(j," ", i,":", monitoring[j,i])
                print("HERE",ids[j,i], " time ", datetime.now())
                #autopilot=False
                if monitoring[j,i]==1:
                    
                    try:
                            #print(camera_id[j])
                        autopilot = stream_save(camera_id[j], ids[j,i], i,j)
                        
                    except Exception as e:
                            #fail_attemp =fail_attemp+1
                            #print(ids[j,i], "Trying again", fail_attemp)
                            #print(e)
                            e1=0
            if i == max(range(r)):
                print("return to dock")
            else:
                print("move forward")
                    

                






                    
