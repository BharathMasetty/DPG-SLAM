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

#%% This part of the code is just for loading data and creating arrays 
path = r"C:\Users\Kunal Nalamwar\Desktop\project\data\Active map points" # use your path
all_csv_files_active_map = glob.glob(path + "/*.csv")
all_csv_files_active_map.sort()

path1 = r"C:\Users\Kunal Nalamwar\Desktop\project\data\Active-Static map points" # use your path
all_csv_files_active_static_map = glob.glob(path1 + "/*.csv")
all_csv_files_active_static_map.sort()

path2 = r"C:\Users\Kunal Nalamwar\Desktop\project\data\Dynamic grid map points" # use your path
all_csv_files_dynamic_map = glob.glob(path2 + "/*.csv")
all_csv_files_dynamic_map.sort()

path3 = r"C:\Users\Kunal Nalamwar\Desktop\project\data\Pose graph SLAM" # use your path
all_csv_files_pose_graph_map = glob.glob(path3 + "/*.csv")
all_csv_files_pose_graph_map.sort()

#print(all_csv_files_active_map[0])
#print(np.shape(all_csv_files_active_map))

obj = [[] for i in range(9)]
obj1 = [[] for i in range(9)]
for i in range(1,np.shape(all_csv_files_active_map)[0]):
    df = pd.read_csv(all_csv_files_dynamic_map[i], index_col=None, header=0, skiprows=1 , usecols=[0,1,2])
    df1 = df.to_numpy()
    print(np.shape(df1))
    np.asarray(obj[i].append(df1))
    print(np.shape(obj[i]))
    df2 = pd.read_csv(all_csv_files_active_static_map[i], index_col=None, header=0, skiprows=1 , usecols=[0,1,2])
    df3 = df2.to_numpy()
    print(i)
    print("'''''''''''''''''''''''''''''''''''''")
    print(np.shape(df3))
    np.asarray(obj1[i].append(df3))
    print(np.shape(obj1[i]))
    print(i)
    print("'''''''''''''''''''''''''''''''''''''")
    

#%% This part of the code is used for processing the data arrays created earlier

points = []
for k in range(1,9):
    one = np.asarray(obj[k]).reshape(np.shape(obj[k])[1],np.shape(obj[k])[2])
    two = np.asarray(obj1[k]).reshape(np.shape(obj1[k])[1],np.shape(obj1[k])[2])
    matched_points = 0
    for i in range(0, np.shape(one)[0]):
        for j in range(0, np.shape(two)[0]):
            if (one[i,0] == two[j,0] and one[i,1] == two[j,1] and one[i,2] == two[j,2] ):
                matched_points = matched_points + 1 
    print(matched_points)
    points.append(matched_points)


#%% This part of the code is for plotting data

plt.figure(1)
plt.title('pass number vs matched points', size = 20)
plt.xlabel('pass number', size = 15)
plt.ylabel('matched points', size = 15)
plt.plot(np.arange(1,9),points)
plt.show()
