# -*- coding: utf-8 -*-
"""
Created on Thu Dec  3 22:13:49 2020

@author: Kunal Nalamwar
"""
#This script is used for post processing of the acquired map points for active, dynamic and the ground truth
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import glob
import re
number_passes = 10

#%% This part of the code is just for loading data and creating arrays 
path = r"C:\Users\Kunal Nalamwar\Desktop\project\data_GDC\Active map points GDC" # use your path
all_csv_files_active_map = glob.glob(path + "/*.csv")
all_csv_files_active_map = sorted(all_csv_files_active_map, key=lambda x:float(re.findall("(\d+)",x)[0]))


path1 = r"C:\Users\Kunal Nalamwar\Desktop\project\data_GDC\Active-Static map points GDC" # use your path
all_csv_files_active_static_map = glob.glob(path1 + "/*.csv")
all_csv_files_active_static_map = sorted(all_csv_files_active_static_map, key=lambda x:float(re.findall("(\d+)",x)[0]))


path2 = r"C:\Users\Kunal Nalamwar\Desktop\project\data_GDC\Dynamic grid map points GDC" # use your path
all_csv_files_dynamic_map = glob.glob(path2 + "/*.csv")
all_csv_files_dynamic_map = sorted(all_csv_files_dynamic_map, key=lambda x:float(re.findall("(\d+)",x)[0]))


path3 = r"C:\Users\Kunal Nalamwar\Desktop\project\data_GDC\Pose graph SLAM GDC" # use your path
all_csv_files_pose_graph_map = glob.glob(path3 + "/*.csv")
all_csv_files_pose_graph_map = sorted(all_csv_files_pose_graph_map, key=lambda x:float(re.findall("(\d+)",x)[0]))


ground_truth = pd.read_csv('all_points_ground_truth_gdc.csv', index_col=None, header=0, skiprows=1 , usecols=[0,1,2]).to_numpy()


obj = [[] for i in range(number_passes)]
obj1 = [[] for i in range(number_passes)]
obj2 = [[] for i in range(number_passes)]
obj3 = [[] for i in range(number_passes)]
for i in range(0,number_passes):
    # df = pd.read_csv(all_csv_files_dynamic_map[i], index_col=None, header=0, skiprows=1 , usecols=[0,1,2])
    # df1 = df.to_numpy()
    # np.asarray(obj[i].append(df1))
    df2 = pd.read_csv(all_csv_files_active_static_map[i], index_col=None, header=0, skiprows=1 , usecols=[0,1,2])
    df3 = df2.to_numpy()
    np.asarray(obj1[i].append(df3))
    # df4 = pd.read_csv(all_csv_files_active_map[i], index_col=None, header=0, skiprows=1 , usecols=[0,1,2])
    # df5 = df4.to_numpy()
    # np.asarray(obj2[i].append(df5))
    # df6 = pd.read_csv(all_csv_files_pose_graph_map[i], index_col=None, header=0, skiprows=1 , usecols=[0,1,2])
    # df7 = df6.to_numpy()
    # np.asarray(obj3[i].append(df7))

#%% This part of the code is used for processing the data arrays created earlier

points_percent_GT_active_static = []
for k in range(0,number_passes):
    one = ground_truth
    two = np.asarray(obj1[k]).reshape(np.shape(obj1[k])[1],np.shape(obj1[k])[2])
    matched_points = 0
    for i in range(0, np.shape(one)[0]):
        for j in range(0, np.shape(two)[0]):
            if (one[i,0] == two[j,0] and one[i,1] == two[j,1] and one[i,2] == two[j,2] == 1):
                matched_points = matched_points + 1 
    print(matched_points)
    active_map_points = 0
    for j in range(0, np.shape(two)[0]):
        if (two[j,2] == 1):
            active_map_points = active_map_points + 1 
    print(active_map_points)
    points_percent_GT_active_static.append(matched_points/active_map_points*100)
'''  
points_percent_GT_baseline = []
for k in range(0,number_passes):
    one = ground_truth
    two = np.asarray(obj3[k]).reshape(np.shape(obj3[k])[1],np.shape(obj3[k])[2])
    matched_points = 0
    for i in range(0, np.shape(one)[0]):
        for j in range(0, np.shape(two)[0]):
            if (one[i,0] == two[j,0] and one[i,1] == two[j,1] and one[i,2] == two[j,2] == 1):
                matched_points = matched_points + 1 
    print(matched_points)
    active_map_points = 0
    for j in range(0, np.shape(two)[0]):
        if (two[j,2] == 1):
            active_map_points = active_map_points + 1 
    print(active_map_points)
    points_percent_GT_baseline.append(matched_points/active_map_points*100)

points_percent_GT_dynamic = []
points_percent_dynamic_GT = []
for k in range(0,number_passes):
    one = ground_truth
    two = np.asarray(obj[k]).reshape(np.shape(obj[k])[1],np.shape(obj[k])[2])
    matched_points = 0
    for i in range(0, np.shape(one)[0]):
        for j in range(0, np.shape(two)[0]):
            if (one[i,0] == two[j,0] and one[i,1] == two[j,1] and one[i,2] == two[j,2] == 1):
                matched_points = matched_points + 1 
    print(matched_points)
    active_map_points_dynamic = 0
    for j in range(0, np.shape(two)[0]):
        if (two[j,2] == 1):
            active_map_points_dynamic = active_map_points_dynamic + 1 
    print(active_map_points_dynamic)
    points_percent_GT_dynamic.append(matched_points/active_map_points_dynamic*100)
    points_percent_dynamic_GT.append(matched_points/(np.shape(ground_truth)[0])*100)
'''
#%% This part of the code is for plotting data

plt.figure(1)
plt.title('Ground Truth coverage of active-static points', size = 20)
plt.xlabel('pass number', size = 15)
plt.ylabel('percentage matched points', size = 15)
plt.plot(np.arange(0,number_passes),points_percent_GT_active_static)
plt.show()
'''
plt.figure(2)
plt.title('Ground Truth coverage of baseline', size = 20)
plt.xlabel('pass number', size = 15)
plt.ylabel('percentage matched points', size = 15)
plt.plot(np.arange(0,number_passes),points_percent_GT_baseline)
plt.show()

plt.figure(3)
plt.title('Ground truth coverage of dynamic points', size = 20)
plt.xlabel('pass number', size = 15)
plt.ylabel('percentage matched points', size = 15)
plt.plot(np.arange(0,number_passes),points_percent_GT_dynamic)
plt.show()

plt.figure(4)
plt.title('Dynamic points coverage of ground truth', size = 20)
plt.xlabel('pass number', size = 15)
plt.ylabel('percentage matched points', size = 15)
plt.plot(np.arange(0,number_passes),points_percent_dynamic_GT)
plt.show()
'''