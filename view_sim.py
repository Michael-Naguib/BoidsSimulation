

#Code By Michael Sherif Naguib
# @ University of Tulsa 8/20/19
# ... Open3d only runs 1 time bc internal error ... jupyter needs to be restarted each time ...  
# regular python myFile.py works each time! 

# #imports
print("Getting Imports")
from tkinter import *
from BoidSwarm import BoidSwarm
from Vector import Vector
import time
import tqdm
import math
import copy
import numpy as np
import pickle
import open3d
import time

print("Finished Imports ...  Reading Simulation Data")
#Settings
timeDelay = 0.02 # 0.5=half second ... used to space out each frame in the playback
loop = True
#Read the file
positionFrameHistory = pickle.load(open("position.dat","rb"))
print("Finished Reading Simulation Data")

#Setup 3D views
pcd = open3d.PointCloud()
vis = open3d.Visualizer()
vis.create_window()
vis.add_geometry(pcd)

#Setup the initial frame
pcd.points = open3d.Vector3dVector(positionFrameHistory[0])
pcd.colors = open3d.Vector3dVector([[0,0,0] for i in range(0,len(positionFrameHistory[0]))])

render_option = vis.get_render_option()
render_option.point_size = 5#0.01
print("Bootstraping Simulation Data")
#precompute
for i in range(1,len(positionFrameHistory)):
    positionFrameHistory[i]=open3d.Vector3dVector(positionFrameHistory[i])
#render_option.point_color_option = open3d.visualization.PointColorOption.Normal
to_reset_view_point = True

print("Starting Playback")
i=0
while True: 
    if i < len(positionFrameHistory)-1:
        i+=1
    elif not loop:
        break
    else:
        print("Restarting Playback")
        i=0  
    pcd.points = positionFrameHistory[i]
    vis.update_geometry()
    if to_reset_view_point:
        vis.reset_view_point(True)
        to_reset_view_point = False
    vis.poll_events()
    vis.update_renderer()
    time.sleep(timeDelay)# could be interpreted as frames per second assuming the calculation is instant and retrieval is as well

vis.destroy_window()