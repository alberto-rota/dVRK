#!/usr/bin/env python
import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2
import open3d as o3d
import os
import sensor_msgs.msg as sensor_msgs
import csv
import torch
import datetime

class image_feature:

    def __init__(self):       
        # Open3D elements for point cloud visualization and writing
        self.count = 0
        self.pcd = None
        # Image RESOLUTION (original and new)
        self.w = 1920
        self.h = 1080
        # Frame dictionaries
        self.dictL = {}
        self.dictR = {}
        # Last 2 frames keys
        self.secondKL = None
        self.firstKL = None
        self.secondKR = None
        self.firstKR = None

        self.out_l=[]
        self.out_r=[]


        
    
    # Callback LEFT image
    def callback_left(self, ros_data):
        # Conversion from ROS CompressedImage msg to Opencv image
        np_arr = np.fromstring(ros_data.data, np.uint8)
        kl = int(ros_data.header.stamp.to_nsec() // 10e6)       # CENTESIMI sec from epoch
        self.dictL[kl] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
        # Save keys of the last 2 frames
        self.secondKL = self.firstKL
        self.firstKL = kl     
         

    # Callback RIGHT image
    def callback_right(self, ros_data):
        # Conversion from ROS Compressed Image msg to Opencv image
        np_arr = np.fromstring(ros_data.data, np.uint8)
        kr = int(ros_data.header.stamp.to_nsec() // 10e6)
        self.dictR[kr] = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Save keys of the last 2 frames
        self.secondKR = self.firstKR
        self.firstKR = kr



    #Computation function for 3d reconstruction
    def computation(self):
        if len(self.dictL.keys()) > 130 :
            L = [self.dictL[self.secondKL],self.dictL[self.firstKL]]
            R = [self.dictR[self.secondKR],self.dictR[self.firstKR]]
            self.dictL.clear()
            self.dictR.clear()
            self.dictL[self.secondKL], self.dictL[self.firstKL] = L
            self.dictR[self.secondKR], self.dictR[self.firstKR] = R

        #print(ic.count)
        imgL1 = self.dictL[self.firstKL]
        imgR1 = self.dictR[self.firstKR]
        ic.out_l.write(imgL1)
        ic.out_r.write(imgR1)

        return

if __name__ == '__main__':
    rospy.init_node('reconstruction', anonymous=True)
    rate = rospy.Rate(28)    

    # Class initialization
    ic = image_feature()
    # Work folder path
    work_path = '.'        

    subscriber_left = rospy.Subscriber("/endoscope/raw/left/image_raw/compressed", CompressedImage, ic.callback_left, queue_size = 1)
    subscriber_right = rospy.Subscriber("/endoscope/raw/right/image_raw/compressed", CompressedImage, ic.callback_right, queue_size = 1)  
    
    # Publisher to point cloud topic
    pub_point = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
    size = (ic.w,ic.h)
    ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    name='user_1_'+ts+'_left.avi'
    name2='user_1_'+ts+'_right.avi'
    ic.out_l = cv2.VideoWriter(str(name),cv2.VideoWriter_fourcc(*'DIVX'), 25, size)
    ic.out_r = cv2.VideoWriter(str(name2),cv2.VideoWriter_fourcc(*'DIVX'), 25, size)
    start=time.time()



    while not rospy.is_shutdown() : 

        # Run 3D reconstruction
        if len(ic.dictL.keys()) > 1 :
            ic.count=ic.count+1
            ic.computation()


        rate.sleep()
        # rospy.spin()
    end=time.time()-start
    print(end)

    ic.out_l.release()
    ic.out_r.release()
    print("frame:",ic.count)