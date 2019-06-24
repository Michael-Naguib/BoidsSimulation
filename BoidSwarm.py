'''
# BoidSwarm
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
## Description:
- Modeling a Boids Flock (swarm) according to Separation Alignment & Cohesion and Avoidance Rules
                see: https://www.red3d.com/cwr/boids/
- well... not quite ... I had some ideas along the way:
- boid weight:  most calculations for center of mass (cohesion) assume each boid weighs the same or rather that
                each boid has the same importance... I introduce a weight factor which for a normal boid swarm
                simulation will have no effect if all weights are the same ... just a fun parameter to make a
                random distribution of weights and see what will happen.
'''
#imports
import random
import time
import arcade
import uuid
import math
from Boid import Boid

#Helps create and run a boid swarm
class BoidSwarm():
    #=== internal setup for this class do not modify
    def __init__(self,tkinter_canvas,max_x,max_y):
        '''
        :description: internal class initialization for the swarm
        :param tkinter_canvas: is the reference to the tkinter canvas that the boids will be drawn on
        :param max_x: the maximum world bound in the x direction... ( boid x cordinates always on [0,max_x] )
        :param max_y: the maximum world bound in the y direction... ( boid y cordinates always on [0,max_y] )

        usage:

        bs = BoidSwarm(160,100)
        bs.setup()# you must override this
        bs.init_kinematics()# you must override this

        ... loop
        bs.update_boid_positions

        '''
        #=== Update Rules & Boid Storage
        self.boid_list=[]#      stores all the boid objects; relative order in this list should be preserved!
        self.update_rules=[]#   stores every function that will be passed the boid_list ==> note should return a list of velocity update tuple pairs [(vx,vy)...]
        #Stores the modular wrapping bounds for the spherical surface  (modular 2d space) assumes min_x,min_y = 0
        self.max_y = max_y
        self.max_x = max_x
        #Stores the tkinter canvas instance
        self.tkinter_canvas = tkinter_canvas
        #Stores the Markov Matricies for the relationships between each Update Rule hyper-parameters between every boid (technically its not quite a markov matrix ... as sigma prob != 1 necessarily but it does capture relationship informaiton
        #self.markov_tensor=[]# store each matrix for each function in here ... --> order corresponds to the functions prder in update_rules
        #Stores the Partitioning scheme for each Markov Matrix (i.e  indicies a thru b  have same update rule for [(rnage1,range2...)] for rule P
        #         [ scheme for function 0, scheme for function 1,...]  where a scheme ==> [ {[a,b] ,([r0,r1],[r2,r3],...])} , {[c,d] ,([r0,r1],[r2,r3],...])},... ]  ==> that is a scheme for 1 function
        #         a<b<c<d ==> they are in order ... every partition must be accounted for...
        #self.markov_partitioning_tensor=[]

    #=== (OVERRIDE) setup!! or use this default this is where you build custom types of swarms...)
    def setup(self,quantity=100):
        '''
        :description: Sets up the initial state of the swarm i.e how many boids configured  color ... etc... using [...] update rules and hyper parameters
        '''
        #Add the desired update rules (functions  accept boidlist as param ... returns velocity update ... use a lambda abstraction to bind hyper-parameters
        cohere = lambda boid_list: BoidSwarm.COHESION(boid_list,percent_strength=0.01,modular_space_params=(self.max_x,self.max_y))
        seperate = lambda boid_list: BoidSwarm.SEPARATION(boid_list, percent_influence_range=0.1,modular_space_params=(self.max_x, self.max_y))
        align = lambda boid_list: BoidSwarm.ALIGNMENT(boid_list, percent_strength=0.01,modular_space_params=(self.max_x, self.max_y))
        self.update_rules.extend([cohere,seperate,align])

        #Create the swarm by appending the new boid to self.boid_list: example generate 100 boids with random positions within a certian bound
        max_x=160#       if implementing this in a graphical sense these could be the screen bounds...
        max_y=100#
        for i in range(quantity):
            rand_x = random.randint(0,max_x)#assuming min_x=0   THIS DOES NOT MATTER AS INIT KINEMATICS OVERRIDES these positions...
            rand_y = random.randint(0, max_y)#assuming min_y=0
            self.boid_list.append(Boid(rand_x,rand_y)) #NOTE! velocity inits to a random dydt dxdy each on [-1,1] but if that is too slow for a starting velocity it should be set before appending

    #=== (OVERRIDE) inititilize the kinematics for the boid: starting position and starting velocity
    def init_kinematics(self):
        '''
        :description: this function should iterate over every boid in the self.boid list and set an initial position and
                      velocity
        '''
        #Random strategy:
        for current_boid_index in range(0,len(self.boid_list)):
            #update
            ref=self.boid_list[current_boid_index]
            #init to a random velocity and a random position
            #a constant to make sure starting speed is fast enough
            s = 10
            ref.dydt = random.randint(-s,s)
            ref.dxdt = random.randint(-s,s)
            ref.x = random.randint(ref.scale,self.max_x)# inits a random x between scale and max --> the minumum bound is scale because tkinter cant draw outside canvas...
            ref.y = random.randint(ref.scale,self.max_y)

    #=== essentially advances the next time step
    def update_boid_positions(self):
        '''
        :description: updates all of the velocities and positions of the boids in the boid list according to the
                      list of present update rules stored in update_rules
        '''

        #Make no mistake this one line is a massive calculation....
        velocity_updates = [rule(self.boid_list) for rule in self.update_rules]# [  {v1={x,y},v2},{etc...}  ]
        #print(velocity_updates)

        for current_boid_index in range(0,len(self.boid_list)):
            #intit
            velocity_x = 0
            velocity_y = 0
            #sum for each rule
            for i in range(0,len(velocity_updates)):
                velocity_x += velocity_updates[i][current_boid_index][0]
                velocity_y += velocity_updates[i][current_boid_index][1]
            #update
            ref=self.boid_list[current_boid_index]
            #change position and velocity... (manually)
            ref.dxdt = velocity_x if velocity_x != 0 else ref.dxdt
            ref.dydt = velocity_y if velocity_y != 0 else ref.dydt

            #NOW update the position
            ref._update_position(self.max_x,self.max_y)

    #=== draw the boid swarm
    def draw_swarm(self):
        for boid in self.boid_list:
            boid._draw(self.tkinter_canvas)
    #=== Predefined Update rules! these may be used withine the BoidSwarm ... custom rules can be added...
    @staticmethod
    def COHESION(boid_list,percent_strength=0.01,modular_space_params=False):
        '''
        :description: Calculates the updates to the velocities of each boid in the swarm
                      each boid only considers other boids in its own influence_radius
                      each boid is excluded when calculating its own velocity update (e.g its own center of mass is not taken into account)
        :param boid_list: a reference to the list of boids
        :param percent_strength: defaults to 0.01; determines how strong the cohesion takes effect as a percentage 0.01 = 1%
        :param modular_space_params: defines the (max_x,max_y) for the space assuming it loops (is used in the calcuations to determine euclidian distance)
        :return: a list of velocity updates for each boid ... ex. [(vx_0,vy_0),(vx_1,vy_1),...,(vx_n,vy_n)] (order should match the boid it references)
        '''
        velocity_update_list=[]#store the updated velocities

        '''
        Calculate the center of mass for the boid 
            (this would be a great place to accelerate using multiple cores or distribute computation)
        Algorithm: (for each boid)   O(n^2) ouch.... but can be easily run in parallel ^^ 
            (1) compute edge distance between itself and all other boids keeping track of each index associated with edge [edge dist,boid index]
            (2) filter by edge excluding all items where  edge distance > influence_radius and index != itself (the current boid we are computing for)
            (3) calculate velocity update...  v = *percent/n_items)*sigma (weight)*position_vec
        '''
        #define a function for edge distance between two boids (euclidian distance)
        edge_distance = lambda boid0,boid1: BoidSwarm.euclidian_distance(boid0,boid1,modular_space=modular_space_params)

        #Iterate over all the boids in the list to build the velocity out
        for current_boid_index in range(0,len(boid_list)):
            #compute the edge distance and filter keeping a running count and weighted sum
            w_sum_x=0
            w_sum_y=0
            count=0
            for i in range(0,len(boid_list)):
                #Note for future use: if cohesion is a per swarm property that is to say that cohesion exists differently (relative varying ammounts)
                #or not at all between swarms in the latter case the algorithm could be sped up by partitioning the swarm space so it does not have t
                #to iterate over known partitions for which it does not interact....
                if i!=current_boid_index:#ensure we dont include the current boid...
                    #compare the current to every other in the list
                    edge_dist = edge_distance(boid_list[current_boid_index],boid_list[i])
                    #compare the edge distance to influence radius of the current boid
                    if edge_dist <= boid_list[current_boid_index].influence_radius:
                        count+=1
                        w_sum_x+=boid_list[i].x*boid_list[i].weight
                        w_sum_y+=boid_list[i].y*boid_list[i].weight
            #Now multiply weighted sum by percent strength/(number of items =count)    if number of items=0 then assign 0 for the update = no effect on velocity
            v_update_x =  (percent_strength/count)*(w_sum_x) if count !=0 else 0
            v_update_y =  (percent_strength/count)*(w_sum_y) if count !=0 else 0
            #append the result
            velocity_update_list.append((v_update_x,v_update_y))

        #return the velocity updates
        return velocity_update_list
    @staticmethod
    def SEPARATION(boid_list,percent_influence_range=0.5,modular_space_params=False):
        '''
        :description: determines the velocity update for separation of boids
        :param boid_list: a reference to the list of boids
        :param percent_influence_range: determines how far another boid must be to consider for this function  ex influence range =5 ==> if percent=0.4=40% ==> then distance <2 is considered
        :param modular_space_params: defines the (max_x,max_y) for the space assuming it loops (is used in the calcuations to determine euclidian distance)
        :return: a list of velocity updates for each boid ... ex. [(vx_0,vy_0),(vx_1,vy_1),...,(vx_n,vy_n)] (order should match the boid it references)
        '''
        velocity_update_list = []  # store the updated velocities
        # define a function for edge distance between two boids (euclidian distance)
        edge_distance = lambda boid0, boid1: BoidSwarm.euclidian_distance(boid0,boid1,modular_space=modular_space_params)
        # Iterate over all the boids in the list to build the velocity out
        for current_boid_index in range(0, len(boid_list)):
            min_distance = boid_list[current_boid_index].influence_radius*percent_influence_range
            v_update_x = 0
            v_update_y = 0
            for i in range(0,len(boid_list)):
                if (current_boid_index !=i) and edge_distance(boid_list[current_boid_index],boid_list[i])<=min_distance:#ensure it is not itself and make sure it is in the min_diatance range
                    v_update_x = v_update_x - (boid_list[current_boid_index].x-boid_list[i].x)
                    v_update_y = v_update_y - (boid_list[current_boid_index].y-boid_list[i].y)
            velocity_update_list.append((v_update_x,v_update_y))
        return velocity_update_list
    @staticmethod
    def ALIGNMENT(boid_list,percent_strength=1/8,modular_space_params=False):
        '''
        :description: calculates the alignment velocity updates!
        :param percent_strength: the strength of how much a boid is influenced by the aligmnet or perceived velocities of the other boids
        :return:a list of velocity updates for each boid ... ex. [(vx_0,vy_0),(vx_1,vy_1),...,(vx_n,vy_n)] (order should match the boid it references)
        '''
        velocity_update_list = []  # store the updated velocities
        # define a function for edge distance between two boids (euclidian distance)
        # Iterate over all the boids in the list to build the velocity out
        for current_boid_index in range(0, len(boid_list)):
            v_update_x = 0
            v_update_y = 0
            count=0
            for i in range(0, len(boid_list)) :
                proximity=BoidSwarm.euclidian_distance(boid_list[i],boid_list[current_boid_index], modular_space=modular_space_params)
                if (current_boid_index != i) and proximity<boid_list[current_boid_index].influence_radius:
                    v_update_x = v_update_x +boid_list[i].dxdt#x velocity
                    v_update_y = v_update_y +boid_list[i].dydt#y velocity
                    count+=1
            v_update_x= (v_update_x-boid_list[current_boid_index].x)*percent_strength/count if count != 0 else 0
            v_update_y= (v_update_y-boid_list[current_boid_index].y)*percent_strength/count if count != 0 else 0
            velocity_update_list.append((v_update_x, v_update_y))
        return velocity_update_list

    #=== Useful methods
    @staticmethod
    def euclidian_distance(boid0,boid1,modular_space=False):
        '''

        :param boid0: a boid with an x and a y cordinate   ==> boid.x boid.y
        :param boid1: a boid with an x and a y cordinate
        :param modular_space:  defaults to false ==> computes euclidian distance assuming the topography 'wraps' ...
                               pass (x_max,y_max) where it is assumed the world is bounded by min_x=0,min_y=0 and where
                               y_max and x_max are the maximum allowed x and y_positions within the world
        :return: distance as float
        '''
        if not modular_space:
            #Normal implemetation of euclidian distance
            return math.sqrt(math.pow(boid0.x-boid1.x,2)+math.pow(boid0.y-boid1.y,2))
        else:
            #use the equation i derived for modular space euclidian distance   min(|x1-x2|,max_x - |x1-x2|)   same applies to y (note the order is not specific for x2-x1 or x1-x2 but it must be consistant
            #X
            delta_x = abs(boid0.x-boid1.x)
            x_comp = min(delta_x,modular_space[0]-delta_x)#x component
            #Y
            delta_y = abs(boid0.y - boid1.y)
            y_comp = min(delta_y, modular_space[1] - delta_y)#y component
            return math.sqrt(math.pow(x_comp,2)+math.pow(y_comp,2))