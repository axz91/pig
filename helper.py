import pyrealsense2.pyrealsense2 as rs
import cv2
import numpy as np
import os
import imageio
import threading
from pyntcloud import PyntCloud
import tifffile
from realsense_device_manager import DeviceManager, post_process_depth_frame
import time
from helper_functions import cv_find_chessboard, get_chessboard_points_3D, get_depth_at_pixel, convert_depth_pixel_to_metric_coordinate
import scipy.io
import pyrealsense2.pyrealsense2 as rs

class saveDataThread(threading.Thread):
    def __init__(self, threadID, frameset, intr, folder_location, fileName, time_stamp ):
        threading.Thread.__init__(self)
        self.threadID=threadID
        self.frameset=frameset
        self.intr=intr
        self.folder_location = folder_location
        self.fileName = fileName
        self.time_stamp=time_stamp
        

    def run(self):
        folder_location = self.folder_location

        if not os.path.exists(folder_location+"DM"):
            os.makedirs(folder_location+"DM")
        if not os.path.exists(folder_location+"DP"):
            os.makedirs(folder_location+"DP")
        if not os.path.exists(folder_location+"RGB"):
            os.makedirs(folder_location+"RGB")
        if not os.path.exists(folder_location+"IR"):
            os.makedirs(folder_location+"IR")
        depth_threshold=rs.threshold_filter(min_dist=0.15, max_dist=2.5)
        colorizer=rs.colorizer()
        intr=self.intr
        fileName = self.fileName
        t=self.time_stamp
        imgname = str(fileName) + "_"+ str(t.year)+"_"+ str(t.month)+"_"+ str(t.day)+ "_"+str(t.hour)+"_"+str(t.minute) + "_"+str(t.second)+"_"

        frames=self.frameset
        color_frame=frames.get_color_frame()
        depth_frame=frames.get_depth_frame()
        ir_frame=frames.get_infrared_frame()
        #depth image
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        ir_image = np.asanyarray(ir_frame.get_data())
        
        #depth_frame=depth_threshold.process(depth_frame)
        filtered_depth_frame=post_process_depth_frame(depth_frame, temporal_smooth_alpha=0.1, temporal_smooth_delta=50)

        #index=(self.i)
        #print(index)
        #'''
        XX=np.zeros((480,480))
        YY=np.zeros((480,480))
        ZZ=np.zeros((480,480))
       
        coverage = [0]*64
        for y in range(480):
            for x in range(480):
                dist = depth_frame.get_distance(x+80, y)
                #print(dist)
                [X,Y,Z] = convert_depth_pixel_to_metric_coordinate(dist, x,y, intr)
                XX[y,x] = X
                YY[y,x] = Y
                ZZ[y,x] = Z
        obj=np.stack((XX,YY,ZZ))

        
        matfile=folder_location +"DM/" +imgname+ ".mat"
        scipy.io.savemat(matfile, mdict={'out': obj}, oned_as='row')
        #'''

        imageio.imwrite(folder_location+"DP/"+imgname+".png", depth_image)
        #print(color_image.shape)
        imageio.imwrite(folder_location+"IR/"+imgname+".png", ir_image)
        imageio.imwrite(folder_location+"RGB/"+imgname+".png", color_image)

        #print("exiting"+str(self.threadID))
        
