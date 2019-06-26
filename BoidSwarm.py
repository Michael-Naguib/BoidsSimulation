'''
# BoidSwarm
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
## Description:
- Modeling a Boids Flock (swarm) according to Separation Alignment & Cohesion and Avoidance Rules
  see: https://www.red3d.com/cwr/boids/
- The code needed a revision as it ran too slow: this is probably due the the large quantity of O(n^2) algorithms
  this could be improved by changing the simulation ...
- Also admittedly the code was horribly complex --> I have opted to go for basics and then after mastering that, add on
  functionality
'''

#imports
import random
import math
from Vector import Vector

#Helps create and run a boid swarm
class BoidSwarm():
    #=== internal setup for this class do not modify
    def __init__(self,quantity):
        '''
        :description: internal class initialization for the swarm
        :param quantity: the desired number of boids to be created...

        usage:
        bs = BoidSwarm(quantity=100)
        bs.update_boid_positions()
        bs.draw(tkinter_canvas)

        '''
        #=== Update Rules & Boid Storage
        self.boid_list = []

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
    def euclidian_distance(pos_0,pos_1,modular_space=False,squared=False):
        '''

        :param pos_0: a position of type Vector
        :param pos_1: a positionof type Vector
        :param modular_space:  defaults to false ==> computes euclidian distance assuming the topography 'wraps' ...
                               pass a Vector containing the maximmum allowed values
                               where it is assumed the world is bounded by min_x=0,min_y=0 ...etc... and where
                               y_max and x_max are the maximum allowed x and y_positions within the world etc....
        :param squared: defaults to false ==> as sqrt is a expensive operation when computing length ... you can
                        just get the distance squared easily and do a multiplication of the distance you want to compare
                        to and that is more efficient...
        :return: distance as float
        '''
        if not modular_space:
            #Normal implemetation of euclidian distance
            return len(pos_0-pos_1) if not squared else sum((pos_0-pos_1).apply(lambda x: x*x).components)
        else:
            #use the equation i derived for modular space euclidian distance   min(|x1-x2|,max_x - |x1-x2|)   same applies to y (note the order is not specific for x2-x1 or x1-x2 but it must be consistant

            #Compte the seperation of the components
            delta = (pos_0-pos_1).apply(abs)
            #compute distance from boundry
            mod_dist  = modular_space - delta
            #compute the minumum distance for each... ==> distance components
            dist_vect = Vector([min(delta[i],mod_dist[i]) for i in range(0,len(delta.components))])
            #returns the distance between the two n dimensional points in modular space
            return len(dist_vect) if not squared else sum(dist_vect.apply(lambda x:x*x).components)
    @staticmethod
    def calc_distance_map(list_of_positions, modular_space=False,squared=False):
        '''
        :description: uses a caching based approach to calculate a dictionaries of dictionaries containing the position distances:
                          A   to    B         dist
                      {index:{ otherindex: distance,....},....}
        :param list_of_positions: a list of position Vectors [Vector,Vector,...,Vector]
        :modular_space: see euclidian_distance docstring
        squared: see euclidian_distance docstring
        :return: (see description)
        '''
        cache={}
        list_of_dist_dicts=[{} for i in range(0,len(list_of_positions))]
        for i in range(0,len(list_of_positions)):
            cache[i] = {}#put a dictionary in each
            for j in range(0,len(list_of_positions)):
                #if neither key type is in the cache... then we need to calculate the value
                if str(j) not in cache[i]:
                    #Run the 'intensive' calculation
                    dist = BoidSwarm.euclidian_distance(list_of_positions[i],list_of_positions[j],modular_space=modular_space,squared=squared)
                    #ADD to both dictionaries
                    if cache[i]==None:
                        cache[i]={}
                    if cache[j]==None:
                        cache[j]={}
                    cache[j][i] = dist
                    cache[i][j] = dist
                else:
                    pass # it was added to both dictionaries above...
        return cache
