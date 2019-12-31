'''
# Boids Simulation
- Code By Michael Sherif Naguib
- license: MIT
- Date: 11/25/19
- @University of Tulsa
- Description: The main code for configuring and running a simulation ...
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
import copy                                 # for making copies of objects
import colorsys                             # For recoloring the swarm
print("Finished Importing Deps") if LOG else None

#===================================================================================================================
# ============================= Strictly Compute Settings
VISUALIZE = False  # If set to True the simulation is displayeds
TIME_STEP = 0.3  # The change in time to consider when using Euler's Method for approximating the kinematics
MAX_TIME_SEC = 5*60  # Specify the ammount of time to run the simulation for ...
MODULO_WRAP = False  # If true wrap positions by computing the modulo by the world bounds ... (WARN: will overide wall force)
VIEW_ARC = 2 * math.pi / 3  # The perceptual field of view of the boid for the left and right half [0,pi]  pi==> full circle
PRECOMPUTE = True      # Determines whether the simulation should be precomputed then replayed

#============================= Load/Save Settings
SAVE_SIM = "largesim.0.type2"        # When Specified as a string ... the sim is saved under that Name
LOAD_SIM = "./simulations/largesim.0.type1.slim.dat"       # When Specified as a string ... the sim will be loaded from a file and played back
SLIM_SIM = True                   # Saving all the simulation data can be very memory and data intensive ... save only colors and positions ...
                                   # Slim sim will only save the truncated position and frame data .... i.e the framesData format ...
                                   # this setting applies to loading and saving ... will append will have extension .slim.dat
#============================ Visualization Settings
BOID_SCALE = 5          # Specifies the size of the pixel to render for each boid
SHOW_ORIGIN=  True      # If true will put a ball at the origin
SHOW_BOX = True        # If true a mesh box will show the bounding borders of the simulation
FRAME_DELAY = 0.03 if PRECOMPUTE else 0  # Specify a frame delay (useful when precomputing is used)

# =========================== Color Settings
# Specify a function that takes in a Swarm (list of boid objects) and returns the colors for that Swarm:
# A complex function could color the boid by its acceleration or velocity ... this function should update the values
# of each boids   boid.color property! Also note more complex functions may have a longer compute time ...
def black(swarm):
    for b in swarm:
        b.color = (0,0,0)
COLOR_SWARM_FUNCTION = black

# ============================= Boid Settings
BOID_QUANTITY= 800      # The number of boids in the simulation
NEIGHBOR_QUERY_QUANT = 6 # The maximum number of neighbors each boid should consider ... in the wild
                         # starlings which exhibit flocking behavior only consider 6 of their neighbors

#============================ Special Playback only modification
def original(frameDataI):# The coloring of the original sim...
    return frameDataI[1]
def byDevCOP(frameDataI):# Color boids by normalized deviated displacement from center of positions ...
    posSum = np.zeros(3)
    pos = frameDataI[0]
    for p in pos:
        posSum = np.add(posSum, p)
    avgPos = (1 / len(pos)) * posSum
    colors = [[abs(p[i] -(avgPos[i] * 1.0)) if avgPos[i] != 0 else 0 for i in range(3)] for p in pos]
    for indx in range(len(colors)):
        n = np.linalg.norm(colors[indx])
        colors[indx] =  np.array(colors[indx])*(1/n) if n!=0 else colors[indx]
    return colors
def hByDevCOP(frameDataI): # Color hue by deviation from avg length of displacement from COM
    # Calculate the COM
    posSum = np.zeros(3)
    pos = frameDataI[0]
    for p in pos:
        posSum = np.add(posSum, p)
    com = (1 / len(pos)) * posSum
    colors = []
    # For every position calculate the average dist from the COM
    distFromCom =[]
    for indx in range(len(pos)):
        n = np.linalg.norm(np.subtract(pos[indx],com))
        distFromCom.append(n)
    # scale by the maximum distance from com
    s = max(distFromCom)
    s = s if s!=0 else 0.000000001# Some really small term...
    nDistFromCom = [dist/s for dist in distFromCom]
    #Now compute the colors
    colors = [colorsys.hsv_to_rgb(nDist,1,1) for nDist in nDistFromCom]

    return colors
def byPos(frameDataI): # Color by relative maginitude from the origin
    colors = list(frameDataI[1])
    pos = frameDataI[0]
    for indx in range(len(colors)):
        n = np.linalg.norm(pos[indx])
        colors[indx] = np.array(pos[indx]) * (1 / n) if n != 0 else np.array(pos[indx])
    return colors
#============ type 2 data only: type 2 data saves velocity and accel... in framesData
def byMaxAcel(frameDataI):
    assert(len(frameDataI)>2)# ONLY WORKS WITH TYPE 2 frame data sims ...
    #Get the maximum acceleration
    accel = [np.linalg.norm(a) for a in frameDataI[2]]
    ma = max(accel)
    ma = ma if ma !=0 else 0.00000001
    colors = [colorsys.hsv_to_rgb(math.sin(2*math.pi*amag/ma), 1, 1) for amag in accel]
    return colors

RECOLOR_SWARM_FUNC = hByDevCOP

# ============================= World Settings
# Height,Width,Depth Specifies the size of the dimensions of the simulation
HEIGHT = 3000
WIDTH = HEIGHT
DEPTH = WIDTH
DIMENSIONS = [HEIGHT,WIDTH,DEPTH]

# ============================= FORCE SETTINGS
# Specify the Rule Weights and Distances: Cohere Seperate Align (order specified by the order in the force listing)
MAX_RANGE = 200                  # Specify the max distance for the ranges (i.e range 0.5 ==> 0.5* MAX_DIST)
REL_RANGES = [3.4,1,2]                         # Specify the Relative distances of each rule
REL_RULE_WEIGHTS = [0.15,0.5,0.35]                   # Specify the Relative weights of each Rule

# ============================= Bounding Settings
# Bound Kinematics in the simulation
MAX_FORCE = 3*9.81                      # Specify the maximum (magnitude) force that can ever be exerted in the simulation
MAX_ACCEL = MAX_FORCE*4                   # Specify the maximum (magnitude) acceleration that can be achieved in the acceleration
MAX_VEL = 0.25*math.pow(MAX_ACCEL,2)     # Limit the velocity (magnitude)
WALL_FORCE = MAX_FORCE                  # Special Force exempt from MAX_FORCE Restriction to bound the simulation ...
                                        # which grows in proportion to the distance squared deviated from the bounds!
BOUNDS = [MAX_ACCEL,MAX_FORCE,MAX_VEL]

#============================== Init Spawn Setting
MAP_DIM_FUNC = lambda i: 100        # A function that looks at the current size of the max value for dimension
                                    # i and returns the proper range ... for world size use DIMENSIONS[i]

# ============================= Force Listing
# Generate the list of rules for the forces: Cohere,Seperate,Align
FORCES = [
    BoidUtils.BoidUtils.cohereForce,
    BoidUtils.BoidUtils.seperateForce,
    BoidUtils.BoidUtils.alignForce
]

if __name__ == "__main__":
    # ==================================================================================================================
    #                DO NOT MODIFY CODE BELOW HERE IF U ARE JUST WORKING WITH THE SETTINGS
    # Checks
    assert(NEIGHBOR_QUERY_QUANT<BOID_QUANTITY)
    assert(len(REL_RULE_WEIGHTS)==len(REL_RANGES)==len(FORCES))

    #===================================================================================================================
    #                                                  Load a Simulation
    if bool(LOAD_SIM):
        print("Loading the simulation Data") if LOG else None
        with open(LOAD_SIM, 'rb') as f:# Open the file
            # Use Pickle to get the simulation object
            sim = pickle.load(f)


            print("Pre-Computing Data Truncation & Re-Color") if LOG else None
            framesData = []
            if not SLIM_SIM:
                # Overwrite the value of the Dimension Variable (not saved in slim sims)
                DIMENSIONS = sim["DIMENSIONS"]
                for indx in range(len(sim["BOIDFRAMES"])):
                    # Truncate the data for display
                    p = [b.pos[0:3] for b in sim["BOIDFRAMES"][indx]]
                    c = [b.color for b in sim["BOIDFRAMES"][indx]]
                    # Re-Color the swarm if desired
                    colors = RECOLOR_SWARM_FUNC(c)
                    # Append the frame
                    framesData.append([p, colors])
            else:
                framesData = sim

                # Recolor the frame ...
                for frameIndx in range(len(framesData)):
                    framesData[frameIndx][1] = RECOLOR_SWARM_FUNC(framesData[frameIndx])
            print("Loaded Data, Starting Playback")
            vis = VisualizeSqarm.VisualizeSwarm(frameDelay=FRAME_DELAY, box=(DIMENSIONS if SHOW_BOX else False), pointSize=BOID_SCALE,addSphereAtOrigin=SHOW_ORIGIN)
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
        randPos = [random.random()*MAP_DIM_FUNC(i) for i in range(len(DIMENSIONS))]#*DIMENSIONS[i]
        # Create the boid
        # (IMPLICIT in construction) Use the default mass & none type
        b = Boid.Boid(randPos,randVel,randAccel)
        # Append the boid
        boids.append(b)
    # Color the swarm
    COLOR_SWARM_FUNCTION(boids)

    # compute the absolute values for the weights and distances
    absolute_ranges = BoidUtils.BoidUtils.absoluteValsFromRelative(REL_RANGES,MAX_RANGE)
    absolute_weights = BoidUtils.BoidUtils.absoluteValsFromRelative(REL_RULE_WEIGHTS,1)

    # Create a variable to save the state of the simulation at each timestep: when we save a simulation
    # the boids list and the following states will be saved ... Relevant info concerning the settings
    # is also added ... BOIDFRAMES is a list where each entry in the list will be a copy of the pop
    # at each timestep...
    sim = {
        "BOIDFRAMES":[],
        "DIMENSIONS":DIMENSIONS,
        "TIME_STEP":TIME_STEP,
        "MODULO_WRAP":MODULO_WRAP,
        "VIEW_ARC":VIEW_ARC,
        "BOID_QUANTITY":BOID_QUANTITY,
        "NEIGHBOR_QUERY_QUANT":NEIGHBOR_QUERY_QUANT,
        "MAX_FORCE":MAX_FORCE,
        "MAX_ACCEL":MAX_ACCEL,
        "MAX_VEL":MAX_VEL,
        "WALL_FORCE":WALL_FORCE,
        "MAX_RANGE":MAX_RANGE,
        "REL_RANGES":REL_RANGES,
        "REL_RULE_WEIGHTS":REL_RULE_WEIGHTS
    }

    # If we precompute the simulation we will need to store the frames ... truncating etc will be
    # considered a task for precomputing ...
    framesData = []

    # Create the visualizer object
    vis= None
    if VISUALIZE and not PRECOMPUTE:
        # The following line will create a window if it is called so only call it if we want to visualize the sim
        vis = VisualizeSqarm.VisualizeSwarm(frameDelay=FRAME_DELAY,box= (DIMENSIONS if SHOW_BOX else False),pointSize=BOID_SCALE,addSphereAtOrigin=SHOW_ORIGIN)


    #===================================================================================================================
    # Save the start time of the simulation...
    startTime = time.time()
    print("Beginning Simulation & Running for {0}s".format(MAX_TIME_SEC)) if LOG else None
    print("Pre-Computing the simulation") if LOG and PRECOMPUTE else None

    # Create a progress bar to tick in seconds for the simulation
    pbar = None
    pbar = tqdm.tqdm(total=MAX_TIME_SEC) if LOG else None
    currentTime = time.time()
    quantizedTime = 1 # 1 Unit of time in the progress bar (1 second)
    prevTime = currentTime  # The last time the progres bar was updated
    # Run the simulation until the desired duration is reached: current time -start is less than the desired time
    while currentTime-startTime < MAX_TIME_SEC:
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
        # Color the swarm:
        COLOR_SWARM_FUNCTION(boids)
        if not SLIM_SIM:
            sim["BOIDFRAMES"].append(copy.deepcopy(boids))
        # Truncate the data for display
        p = [b.pos[0:3] for b in boids]
        c = [b.color for b in boids]
        a = [b.accel[0:3] for b in boids]
        v = [b.vel[0:3] for b in boids]
        # Extract the frame data needed
        frameData = [p,c,a,v]
        # Visualize as soon as the data for a frame is computed
        if VISUALIZE and not PRECOMPUTE:
            vis.tick(frameData)
        # Visualize but precompute it ... or save
        if SLIM_SIM or (VISUALIZE and PRECOMPUTE) or bool(SAVE_SIM):
            framesData.append(frameData)
        # ==== Update the Time counter and progress bar...
        currentTime = time.time()
        if currentTime - prevTime > quantizedTime and LOG:
            pbar.update(1)
            prevTime = currentTime
    # If the progress bar was used ... close it
    pbar.close() if LOG else None

    # Manually destroy the window when using tick
    if VISUALIZE and not PRECOMPUTE:
        vis.destroyWindow()
    # If we wanted to precompute then we need to now run the sim
    if VISUALIZE and PRECOMPUTE:
        print("Finished Precompute Starting Simulation Playback") if LOG else None
        vis = VisualizeSqarm.VisualizeSwarm(frameDelay=FRAME_DELAY, box=DIMENSIONS if SHOW_BOX else False, pointSize=BOID_SCALE,addSphereAtOrigin=SHOW_ORIGIN)
        vis.runFrames(framesData)
    # If we want to save the sim ...
    if bool(SAVE_SIM):
        try:
            fileName = SAVE_SIM + str("" if not SLIM_SIM else ".slim")+".dat"
            print("Saving to file named: {0}".format(fileName)) if LOG else None
            sim = sim if not SLIM_SIM else framesData
            pickle.dump(sim, open(str(fileName), "wb"))
        except BaseException as e:
            print("ERROR saving to file: {0}".format(e)) if LOG else None