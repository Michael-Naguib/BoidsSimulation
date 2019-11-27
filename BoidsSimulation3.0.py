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
import math                                 # Python's library for math functions etc,,,
import random                               # Python's library for random numbers
import tqdm                                 # A progress logger bar library for iteration etc...
import time                                 # Python Library for time related things
import open3d                               # A library for LIDAR point cloud visualization and so much more 1k points
import numpy as np                          # a library for scientific computing
import VisualizeSqarm                       # Code for visualizing the swarm using open3d ...
import Boid                                 # Code for a basic boid
import BoidUtils                            # Utilities for the simulation
import pickle                               # Code for serializing and saving python objects
import sys                                  # for exiting early
print("Finished Importing Deps") if LOG else None


if __name__ == "__main__":
    #===================================================================================================================
    #============================= SIM TYPE SETTINGS
    SAVE_SIM = "simPlayback.dat"         # When Specified as a string ... the sim is saved under that Name
    LOAD_SIM = False        # When Specified as a string ... the sim will be loaded from a file and played back
    BOID_SCALE = 5          # Specifies the size of the pixel to render for each boid
    VISUALIZE = True        # If set to True the simulation is displayed
    PRECOMPUTE = False      # Determines whether the simulation should be precomputed then replayed
    TIME_STEP = 0.3         # The change in time to consider when using Euler's Method for approximating the kinematics
    MAX_TIME_SEC = 45       # Specify the ammount of time to run the simulation for ...
    MODULO_WRAP = False     # If true wrap positions by computing the modulo by the world bounds ... (WARN: will overide wall force)
    SHOW_ORIGIN=  True      # If true will put a ball at the origin
    VIEW_ARC = 2*math.pi/3  # The perceptual field of view of the boid for the left and right half [0,pi]  pi==> full circle
    FRAME_DELAY = 0.02 if PRECOMPUTE else 0  # Specify a frame delay (useful when precomputing is used)

    # ============================= WORLD SETTINGS
    # Height,Width,Depth Specifies the size of the dimensions of the simulation
    HEIGHT = 1600
    WIDTH = HEIGHT
    DEPTH = WIDTH
    DIMENSIONS = [HEIGHT,WIDTH,DEPTH]

    # ============================= BOID SETTINGS
    BOID_QUANTITY= 200       # The number of boids in the simulation
    NEIGHBOR_QUERY_QUANT = 6 # The maximum number of neighbors each boid should consider ... in the wild
                             # starlings which exhibit flocking behavior only consider 6 of their neighbors
    # ============================= BOUNDING SETTINGS
    # Bound Kinematics in the simulation
    MAX_FORCE = 2*9.81                      # Specify the maximum (magnitude) force that can ever be exerted in the simulation
    MAX_ACCEL = MAX_FORCE                   # Specify the maximum (magnitude) acceleration that can be achieved in the acceleration
    MAX_VEL = 0.5*math.pow(MAX_ACCEL,2)     # Limit the velocity (magnitude)
    WALL_FORCE = MAX_FORCE                  # Special Force exempt from MAX_FORCE Restriction to bound the simulation ...
                                            # which grows in proportion to the distance squared deviated from the bounds!
    BOUNDS = [MAX_ACCEL,MAX_FORCE,MAX_VEL]

    # ============================= FORCE LISTING
    # Generate the list of rules for the forces: Cohere,Seperate,Align
    FORCES = [
        BoidUtils.BoidUtils.cohereForce,
        BoidUtils.BoidUtils.seperateForce,
        BoidUtils.BoidUtils.alignForce
    ]

    # ============================= FORCE SETTINGS
    # Specify the Rule Weights and Distances: Cohere Seperate Align (order specified by the order in the force listing)
    MAX_RANGE = min(DIMENSIONS)                  # Specify the max distance for the ranges (i.e range 0.5 ==> 0.5* MAX_DIST)
    REL_RANGES = [3,1,2]                         # Specify the Relative distances of each rule
    REL_RULE_WEIGHTS = [2,2,1]                   # Specify the Relative weights of each Rule

    # ============================= COLOR SETTINGS
    # Specify a function that takes in a boid and returns the color for that boid: A complex function could color the
    # boid by its acceleration or velocity ...
    def black(boid):
        return (0,0,0)
    COLOR_FUNCTION = black

    # ==================================================================================================================
    # Checks
    assert(NEIGHBOR_QUERY_QUANT<BOID_QUANTITY)
    assert(len(REL_RULE_WEIGHTS)==len(REL_RANGES)==len(FORCES))

    #===================================================================================================================
    #                                                  Load a Simulation
    if bool(LOAD_SIM):
        print("Loading the simulation Data") if LOG else None
        with open(LOAD_SIM, 'rb') as f:
            framesData = pickle.load(f)
            print("Loaded Data, Starting Playback")
            vis = VisualizeSqarm.VisualizeSwarm(frameDelay=FRAME_DELAY, box=DIMENSIONS, pointSize=BOID_SCALE,addSphereAtOrigin=SHOW_ORIGIN)
            vis.runFrames(framesData)
        sys.exit(0)# Exit early ...
    # ==================================================================================================================
    #                                                  Compute a Simulation
    # Generate the initial state of the boids ...
    boids = []
    for i in range(BOID_QUANTITY):
        # Init a random random velocity for each dimension: consider positive and negative velocities where
        # each component is always less than any of the other components...
        randVel = [ (1-2*random.random())*(1/len(DIMENSIONS))*math.sqrt(MAX_VEL) for i in range(len(DIMENSIONS))]
        # Init a random random Acceleration for each dimension: consider positive and negative accelerations where
        # each component is always less than any of the other components...
        randAccel = [(1-2*random.random())*(1/len(DIMENSIONS))*math.sqrt(MAX_ACCEL) for i in range(len(DIMENSIONS))]
        # Init a random position
        randPos = [random.random()*DIMENSIONS[i] for i in range(len(DIMENSIONS))]
        # Create the boid
        # (IMPLICIT in construction) Use the default mass & none type
        b = Boid.Boid(randPos,randVel,randAccel)
        # Set its color
        b.color = COLOR_FUNCTION(b)
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
    # Run the simulation until the desired duration is reached
    while time.time()-startTime < MAX_TIME_SEC:
        #===== Compute the KD tree
        # Get the positions of the boids: (ORDER SPECIFIC)
        positions = [b.pos for b in boids]
        # Construct the KD tree for the boids
        tree = KDTree(positions)
        # Query the Distances and neighbors for every boid (ORDER SPECIFIC)
        for indx in range(len(boids)):
            # We could construct 3 KD trees ... one for each force but we can get away with constructing only one
            # and querying based off the max distance used of all the forces then checking the distances to ensure they
            # meet the dist criteria for the other force functions
            # For the current agent (position) query a limited number of neighbors within the max of the ranges
            dist,loc = tree.query(positions[indx],k=NEIGHBOR_QUERY_QUANT,distance_upper_bound = max(absolute_ranges))
            # Missing neighbors are specified using infinity ... remove them one u hit inf the rest are inf so break
            shortLoc=[]
            shortDist=[]
            for i in range(len(loc)):
                if dist[i]==float('inf'):
                    break
                else:
                    shortLoc.append(loc[i])
                    shortDist.append(dist[i])
            # assign the neighbors and their distances to the boid..
            boids[indx].neighborsDist = list(zip(shortLoc,shortDist))
            #===== Compute the force updates
            forceSum = np.zeros(len(DIMENSIONS))

            # Iterate over each of the different force functions calculating the updates
            for fIndx in range(len(FORCES)):

                forceVal = FORCES[fIndx](# Get the fIndx'th force and calculate it
                                          boids[indx], boids, loc, dist,
                                          absolute_ranges[fIndx],  # on the settings for the fIndx'th force
                                          max_vel = MAX_VEL,
                                          max_accel = MAX_ACCEL,
                                          arcView = VIEW_ARC
                                      )
                # Weight the force and add it to the force sum
                forceSum = np.add(forceSum, forceVal*absolute_weights[fIndx])


            #===== Limit the force update (1st time)
            forceSum = BoidUtils.BoidUtils.limitVect(forceSum,MAX_FORCE)

            #===== Add the bounding force
            if not MODULO_WRAP:
                # If modulo wrapping is not used then we create a wall force that grows in proportion to the distance
                # squared deviated from the bounds of each component
                boundForce = np.zeros(len(DIMENSIONS))
                for compIndx in range(len(boids[indx].pos)):
                    if boids[indx].pos[compIndx] <= 0:
                        boundForce[compIndx] = WALL_FORCE* math.pow(boids[indx].pos[compIndx],2)
                    elif boids[indx].pos[compIndx] >= DIMENSIONS[compIndx]:
                        boundForce[compIndx] = -WALL_FORCE* math.pow(boids[indx].pos[compIndx]-DIMENSIONS[compIndx],2)
                forceSum = np.add(forceSum,boundForce)
                # ===== Limit the force update (2nd time)
                forceSum = BoidUtils.BoidUtils.limitVect(forceSum, MAX_FORCE)

            #===== Euler's Method:
            # Calculate Acceleration: F=Ma sometimes XD ==> F/M = a
            accel = (1/boids[indx].mass)*forceSum
            # Limit the acceleration
            boids[indx].accel = BoidUtils.BoidUtils.limitVect(accel,MAX_ACCEL)
            # Calculate Velocity
            vel = np.add(boids[indx].vel,boids[indx].accel*TIME_STEP)
            # Limit the Velocity
            boids[indx].vel = BoidUtils.BoidUtils.limitVect(vel,MAX_VEL)
            # Calculate the Updated position
            pos = np.add(boids[indx].pos,boids[indx].vel*TIME_STEP)

            # Modulo Wrap: wrap the boids around the world corners if desired instead of using a wall force
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
        except BaseException as e:
            print("ERROR saving to file: {0}".format(e)) if LOG else None