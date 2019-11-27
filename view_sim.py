

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
from open3d import open3d # <-- Time Lost here: 3 WEEKS Why: weird installation procedure for open3d .... what worked: manually delete open3d in\
# in the python site packages and reinstall using an admin conda command prompt ... also the docs i read for this were totally wack... but it worked on my old pc
# idk why ... i must have installed a previous version where those docs were valid!
import random
import time
print(" Finished Imports ... reading file")
#Settings
timeDelay = 0.02 # 0.5=half second ... used to space out each frame in the playback

#Read the file
positionFrameHistory = pickle.load(open("sim.dat","rb"))
print("Finished Reading the file ... Starting Playback")

HEIGHT = 600.0
WIDTH = round(HEIGHT * 1.6180339) # preserve a nice aspect ratio by using phi= THE GOLDEN RATIO (one of my favorite numbers)
DEPTH = HEIGHT


#Setup 3D views
pcd = open3d.geometry.PointCloud()
vis = open3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)

#The following code adds a persistant wire box mesh around the sim ... 
mesh_box = open3d.geometry.TriangleMesh.create_box(width=WIDTH,height=HEIGHT,depth=DEPTH)
wireBox = open3d.geometry.LineSet.create_from_triangle_mesh(mesh_box)
vis.add_geometry(wireBox)

#Setup the initial frame
pcd.points = open3d.utility.Vector3dVector(positionFrameHistory[0])
pcd.colors = open3d.utility.Vector3dVector([[0,0,0] for i in range(0,len(positionFrameHistory[0]))])

render_option = vis.get_render_option()
render_option.point_size = 5#0.01

# Create a way to adjust the color of the points by their velocity
frameColors = []
print("Precomputing Colors")
#Add one extra because the first frame is init....
frameColors.append(open3d.utility.Vector3dVector([[0,0,0] for i in range(0,len(positionFrameHistory[0]))]))
for i in range(1,len(positionFrameHistory)):
    #For each frame
    if i==1:
        frameColors.append(open3d.utility.Vector3dVector([[0,0,0] for i in range(0,len(positionFrameHistory[0]))]))
    else:
        colors = []
        for j in range(len(positionFrameHistory[i])):
            #Get the velocity
            c = list(map(lambda x: x[0]-x[1],list(zip(positionFrameHistory[i][j],positionFrameHistory[i-1][j]))))
            #Normalize it

            l = math.sqrt(sum([n*n for n in c]))
            nc = [n/l for n in c]
            colors.append(nc)
        frameColors.append(open3d.utility.Vector3dVector(colors))





#precompute
for i in range(1,len(positionFrameHistory)):
    positionFrameHistory[i]=open3d.utility.Vector3dVector(positionFrameHistory[i])
#render_option.point_color_option = open3d.visualization.PointColorOption.Normal
to_reset_view_point = True
for i in range(1,len(positionFrameHistory)):    


    pcd.points = positionFrameHistory[i]
    pcd.colors = frameColors[i]
    vis.update_geometry()
    if to_reset_view_point:
        vis.reset_view_point(True)
        to_reset_view_point = False
    vis.poll_events()
    vis.update_renderer()
    time.sleep(timeDelay)# could be interpreted as frames per second assuming the calculation is instant and retrieval is as well

vis.destroy_window()

