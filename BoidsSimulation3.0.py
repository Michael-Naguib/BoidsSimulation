'''
# Boid
- Code By Michael Sherif Naguib
- license: MIT
- Date: 11/25/19
- @University of Tulsa
- Description: Refactoring the simulation code
'''
# LOGGING CONSTANT
LOG=True

# Imports
print("Importing Deps") if LOG else None
from scipy.spatial import KDTree as KDTree  # A library for constructing and querying a KDTree
from numba import cuda, jit                 # An optimization library that can speed up code and even run on GPU
from tkinter import *                       # Python's library for creating GUI
import math                                 # Python's library for math functions etc,,,
import random                               # Python's library for random numbers
import tqdm                                 # A progress logger bar library for iteration etc...
import pptk                                 # Static 3d point cloud visualization lib for large 10mil+ datapoints
import time                                 # Python Library for time related things
import open3d                               # A library for LIDAR point cloud visualization and so much more 1k points
import numpy as np                          # a library for scientific computing
import VisualizeSqarm                       # Code for visualizing the swarm using open3d ...
import Boid                                 # Code for a basic boid
import BoidUtils                            # Utilities for the simulation
import pickle                               # Code for serializing and saving python objects
print("Finished Importing Deps") if LOG else None


if __name__ == "__main__":
    #===================================================================================================================
    SAVE_SIM = None      # When Specified as a string ... the sim is saved under that Name
    BOID_SCALE = 5       # Specifies the size of the pixel to render for each boid
    VISUALIZE = True     # If set to True the simulation is displayed
    PRECOMPUTE = False   # Determines whether the simulation should be precomputed then replayed
    TIME_STEP = 0.1      # The change in time to consider when using Euler's Method for approximating the kinematics
    MAX_TIME_SEC = 45    # Specify the ammount of time to run the simulation for ...
    FRAME_DELAY = 0.02 if PRECOMPUTE else 0  # Specify a frame delay (useful when precomputing is used)
    MODULO_WRAP = False  # If true wrap positions by computing the modulo by the world bounds ... (WARN: will overide wall force)
    SHOW_ORIGIN=  True   # If true will put a ball at the origin
    VIEW_ARC = 2*math.pi # The perceptual field of view of the boid TODO: Implement this

    # Height,Width,Depth Specifies the size of the dimensions of the simulation
    HEIGHT = 100
    WIDTH = HEIGHT#round(HEIGHT)# * 1.6180339) # Pick nice aspect ratio by using phi= THE GOLDEN RATIO (one of my favorite numbers)
    DEPTH = WIDTH
    DIMENSIONS = [HEIGHT,WIDTH,DEPTH]

    BOID_QUANTITY= 100       # The number of boids in the simulation
    NEIGHBOR_QUERY_QUANT = 6 # The maximum number of neighbors each boid should consider ... in the wild
                             # starlings which exhibit flocking behavior only consider 6 of their neighbors

    # Bound Kinematics in the simulation
    MAX_FORCE = 9.81        # Specify the maximum (magnitude) force that can ever be exerted in the simulation
    MAX_ACCEL = MAX_FORCE       # Specify the maximum (magnitude) acceleration that can be achieved in the acceleration
    MAX_VEL = math.pow(MAX_ACCL)    # Limit the velocity (magnitude)
    WALL_FORCE = (1/3)*math.sqrt(MAX_FORCE)  # Special Force exempt from MAX_FORCE Restriction to bound the simulation ...
                            # which grows in proportion to the distance squared deviated from the bounds!
    BOUNDS = [MAX_ACCEL,MAX_FORCE,MAX_VEL]

    # Specify the Rule Weights and Distances: Cohere Seperate Align
    MAX_RANGE = np.linalg.norm(DIMENSIONS)       # Specify the max distance for the ranges (i.e range 0.5 ==> 0.5* MAX_DIST)
    REL_RANGES = [3,1,2]                         # Specify the Relative distances of each rule
    REL_RULE_WEIGHTS = [1,6,1]                   # Specify the Relative weights of each Rule

    # Specify a function that takes in a boid and returns the color for that boid: A complex function could color the
    # boid by its acceleration or velocity ...
    def black(boid):
        return (0,0,0)
    COLOR_FUNCTION = black

    # ==================================================================================================================
    #Checks
    assert(NEIGHBOR_QUERY_QUANT<BOID_QUANTITY)
    assert(len(REL_RULE_WEIGHTS)==len(REL_RANGES)==len(DIMENSIONS))
    # ==================================================================================================================

    # Generate the initial state of the boids ...
    boids = []
    for i in range(BOID_QUANTITY):
        # Init a random random velocity for each dimension: consider positive and negative velocities where
        # each component is always less than any of the other components... * a dampening factor...
        randVel = [ (1-2*random.random())*(1/len(DIMENSIONS))*math.sqrt(MAX_VEL) for i in range(len(DIMENSIONS))]
        # Init a random random Acceleration for each dimension: consider positive and negative accelerations where
        # each component is always less than any of the other components... * a dampening factor...is added
        randAccel = [(1-2*random.random())*(1/len(DIMENSIONS))*math.sqrt(MAX_ACCEL) for i in range(len(DIMENSIONS))]
        # init a random position in the worldspace
        randPos = [random.random()*DIMENSIONS[i] for i in range(len(DIMENSIONS))]
        # Create the boid
        b = Boid.Boid(randPos,randVel,randAccel)
        # Set its color
        b.color = COLOR_FUNCTION(b)

        # Use the default mass & none type
        # Append the boid
        boids.append(b)

    # compute the absolute values for the weights and distances
    absolute_ranges = BoidUtils.BoidUtils.absoluteValsFromRelative(REL_RANGES,MAX_RANGE)
    absolute_weights = BoidUtils.BoidUtils.absoluteValsFromRelative(REL_RULE_WEIGHTS,1)

    # Setup storage to save the frames of the boids ... precompute so that it is in the format that
    # Visualizer expects ...
    framesData = []

    # Create the visualizer object
    vis= None
    if VISUALIZE and not PRECOMPUTE:
        vis = VisualizeSqarm.VisualizeSwarm(frameDelay=FRAME_DELAY,box=DIMENSIONS,pointSize=BOID_SCALE,addSphereAtOrigin=SHOW_ORIGIN)

    #===================================================================================================================
    # Save the start time of the simulation...
    startTime = time.time()
    print("Beginning Simulation & Running for {0}s".format(MAX_TIME_SEC)) if LOG else None
    print("Precomputing the simulation") if LOG and PRECOMPUTE else None
    while time.time()-startTime < MAX_TIME_SEC:
        #===== Compute the KD tree
        # Get the positions of the boids: (ORDER SPECIFIC)
        positions = [b.pos for b in boids]
        # Construct the KD tree for the boids
        tree = KDTree(positions)
        # Query the Distances and neighbors for every boid (ORDER SPECIFIC)
        for indx in range(len(boids)):
            #TODO: technically we should construct 3 KD trees but we can get away with constructing only one
            # and querying based off the max distance then checking the distances to ensure they meet the dist
            # criteria for the other functions...
            dist,loc = tree.query(positions[indx],k=NEIGHBOR_QUERY_QUANT,distance_upper_bound = max(absolute_ranges))
            # Missing neighbors are specified using infinity ... remove them
            loc = [loc[i] for i in range(len(loc)) if dist[i]!=float('inf')]
            dist = [dist[i] for i in range(len(dist)) if dist[i]!=float('inf')]
            #===== Compute the force updates
            forceSum = np.zeros(len(DIMENSIONS))

            forceSum = np.add(forceSum,
                              BoidUtils.BoidUtils.cohereForce(boids[indx], boids, loc, dist,
                                                              absolute_ranges[0], absolute_weights[0]))
            forceSum = np.add(forceSum,
                              BoidUtils.BoidUtils.seperateForce(boids[indx], boids, loc, dist,
                                                              absolute_ranges[1], absolute_weights[1]))
            forceSum = np.add(forceSum,
                              BoidUtils.BoidUtils.alignForce(boids[indx], boids, loc, dist,
                                                              absolute_ranges[2], absolute_weights[2]))

            #===== Limit the force update
            mag = np.linalg.norm(forceSum)
            if mag > MAX_FORCE:
                forceSum = (MAX_FORCE/mag)*forceSum
            #===== Add the bounding force
            '''
            This force grows in proportion to the distance^2 deviated from the bounds 
            for each component
            '''
            if not MODULO_WRAP:
                boundForce = np.zeros(len(DIMENSIONS))
                for compIndx in range(len(boids[indx].pos)):
                    if boids[indx].pos[compIndx] <= 0:
                        boundForce[compIndx] = WALL_FORCE* math.pow(boids[indx].pos[compIndx],2)
                    elif boids[indx].pos[compIndx] >= DIMENSIONS[compIndx]:
                        boundForce[compIndx] = -WALL_FORCE* math.pow(boids[indx].pos[compIndx]-DIMENSIONS[compIndx],2)
                forceSum = np.add(forceSum,boundForce)

            #===== Euler's Method:
            # Calculate Acceleration: F=Ma sometimes XD ==> F/M = a
            accel = (1/boids[indx].mass)*forceSum
            # limit the accel
            accel_mag = np.linalg.norm(accel)
            if accel_mag!=0:# make sure not div by zer0
                accel = (MAX_ACCEL/accel_mag)*accel if accel_mag > MAX_ACCEL else accel
            boids[indx].accel = accel
            # Calculate Velocity
            vel = np.add(boids[indx].vel,boids[indx].accel*TIME_STEP)
            # limit the velocity
            vel_mag = np.linalg.norm(vel)
            if vel_mag!=0:# make sure not div by zer0
                vel = (MAX_VEL/vel_mag)*vel if vel_mag > MAX_VEL else vel
            boids[indx].vel = vel
            # Calculate the Updated position
            pos = np.add(boids[indx].pos,boids[indx].vel*TIME_STEP)

            # MODULO WRAP...
            if MODULO_WRAP:
                pos = np.array([pos[i]%DIMENSIONS[i] for i in range(len(DIMENSIONS))])
            boids[indx].pos = pos



            #===== Update the color of the boid according to the color func...
            boids[indx].color = COLOR_FUNCTION(boids[indx])

        # Format the data for display
        colors = open3d.utility.Vector3dVector([b.color for b in boids])
        pos = open3d.utility.Vector3dVector([b.pos[0:3] for b in boids])
        frameData = [pos,colors]

        # Visualize as soon as the data for a frame is computed
        if VISUALIZE and not PRECOMPUTE:
            vis.tick(frameData)
        # Visualize but precompute it ... or save
        if VISUALIZE and PRECOMPUTE or bool(SAVE_SIM):
            framesData.append(frameData)

    # Manually destroy the window when using tick
    if VISUALIZE and not PRECOMPUTE:
        vis.destroyWindow()
    # If we wanted to precompute then we need to now run the sim
    if VISUALIZE and PRECOMPUTE:
        print("Finished Precompute Starting Simulation Playback") if LOG else None
        vis = VisualizeSqarm.VisualizeSwarm(frameDelay=FRAME_DELAY, box=DIMENSIONS, pointSize=BOID_SCALE,addSphereAtOrigin=SHOW_ORIGIN)
        vis.runFrames(framesData)
    # If we want to save the sim ...
    if bool(SAVE_SIM):
        try:
            print("Saving to file named: {0}".format(SAVE_SIM)) if LOG else None
            pickle.dump(framesData, open(str(SAVE_SIM), "wb"))
        except:
            print("ERROR saving to file") if LOG else None









    # Visualize,Save,Both
    #loop













