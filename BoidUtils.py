'''
# Boid Utils
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/27/19
- @University of Tulsa
## Description:
- This file holds a lot of usefull Boid Specific STATIC methods for calculations....
'''

#imports
from Vector import Vector
import numpy as np
import math
class BoidUtils:
    def __init__(self):
        pass
    @staticmethod
    def absoluteValsFromRelative(relValues,maxValue):
        '''
        :description:        Compute absolute values given a relative set of values and a maximim:
                             i.e rescale the weights according to size up to the max

                             normalizedVect(relValues)*maxValue
        :param relWeights:   the values to convert to absolute weights [1,1,1] & max=5 ==> [5,5,5]
        :param maxWeight:    the value to scale by
        :return: the absolute values
        '''
        # Each value becomes its fraction of the average
        s = sum(relValues)
        assert(s!=0)
        absoluteValues = [relValue*maxValue*(1/s) for relValue in relValues]
        return absoluteValues

    @staticmethod
    def cohereForce(boid,boids,neighbors,distances,distance,weight,dim=3):
        '''
        :description:       Calculates the coherence force update for the boids sim for a particular boid
        :param boids:       a list of all the boids
        :param neighbors:   a list of indicies of the neighbors of the boid (implicit) we are computing for
        :param distances:   the distances coresponding to that boid to the other nearest boids...
        :param distance:    the max distance for which this rule is valid
        :param weight:      the relative strength of this force...
        :param dim:         the dimension for the vectors...
        :return:            vector representing the force update
        '''
        pos_sum = np.zeros(dim)
        count = 0
        # For every Neighbor:
        for indx in range(len(neighbors)):
            # Check to make sure it meets the distance requirements
            if distances[indx]> distance:
                continue# Goto the next
            # otherwise
            pos_sum = np.add(pos_sum,boids[neighbors[indx]].pos*(1/boids[neighbors[indx]].mass))
        # If there were no force updates return 0 vect
        if count==0:
            return pos_sum
        # otherwise
        target = pos_sum*(1/count)
        steer = np.subtract(np.subtract(target,boid.pos),boid.vel)
        return steer*weight

    @staticmethod
    def seperateForce(boid,boids,neighbors,distances,distance,weight,dim=3):
        '''
        :description:       Calculates the seperation force update for the boids sim for a particular boid
        :param boids:       a list of all the boids
        :param neighbors:   a list of indicies of the neighbors of the boid (implicit) we are computing for
        :param distances:   the distances coresponding to that boid to the other nearest boids...
        :param distance:    the max distance for which this rule is valid
        :param weight:      the relative strength of this force...
        :param dim:         the dimension for the vectors...
        :return:            vector representing the force update
        '''
        pos_sum = np.zeros(dim)
        count = 0
        # For every Neighbor:
        for indx in range(len(neighbors)):
            # Check to make sure it meets the distance requirements
            if distances[indx]> distance:
                continue# Goto the next
            # otherwise
            # calculate current separation distance direction and weight by edge distance
            edge_dist = distances[indx]
            sep_norm = np.subtract(boid.pos,boids[neighbors[indx]].pos) * (1/math.pow(edge_dist,2))#TODO: catch div by zero...
            pos_sum = np.add(pos_sum,sep_norm*boids[neighbors[indx]].mass)

        # If there were no force updates return 0 vect
        if count==0:
            return pos_sum
        # otherwise
        target = pos_sum*(1/count)
        steer =  np.subtract(target,boid.vel)
        return steer*weight
    @staticmethod
    def alignForce(boid,boids,neighbors,distances,distance,weight,dim=3):
        '''
        :description:       Calculates the alignment force update for the boids sim for a particular boid
        :param boids:       a list of all the boids
        :param neighbors:   a list of indicies of the neighbors of the boid (implicit) we are computing for
        :param distances:   the distances coresponding to that boid to the other nearest boids...
        :param distance:    the max distance for which this rule is valid
        :param weight:      the relative strength of this force...
        :param dim:         the dimension for the vectors...
        :return:            vector representing the force update
        '''
        vel_sum = np.zeros(dim)
        count = 0
        # For every Neighbor:
        for indx in range(len(neighbors)):
            # Check to make sure it meets the distance requirements
            if distances[indx]> distance:
                continue# Goto the next
            # otherwise
            vel_sum = np.add(vel_sum,distances[indx]*(1/boids[neighbors[indx]]))
        # If there were no force updates return 0 vect
        if count==0:
            return vel_sum
        # otherwise
        target = vel_sum*(1/count)
        steer = np.add(target,boid.vel)
        return steer*weight
    #=== Predefined Update rules! these may be used withine the BoidSwarm ... custom rules can be added...
    @staticmethod
    def Cohere(boid,boid_list,distance_map,weight,ranger,max_neighbor=10):
        '''
        :description: calculates the weighted coherernce force for a given boid based upon the boid list and distance map etc...
        :param boid: the current boid for which the calculation is being done....
        :param boid_list: a list containing all of the boids
        :param distance_map: a dictionary of key=index, value=distance for every boid in the boid_list
        :param weight:  the weight for this rule
        :param range:   the range for which this rule applies
        :return: the force vector...
        '''
        # store
        pos_sum = Vector.zeros(dim=boid.pos.shape())
        count = 0

        #iterate over the distances in the map...
        for key in distance_map:
            # Note for future use: if cohesion is a per swarm property that is to say that cohesion exists differently (relative varying ammounts)
            # or not at all between swarms in the latter case the algorithm could be sped up by partitioning the swarm space so it does not have t
            # to iterate over known partitions for which it does not interact....
            edge_dist = distance_map[key]
            # compare the edge distance to influence radius of the current boid
            if 0<edge_dist and edge_dist <= ranger:#  if the edge dist = 0 ==> it is itself so exclude....
                count += 1
                pos_sum = pos_sum + boid_list[key].pos#sum the positions
            if  bool(max_neighbor) and count >=max_neighbor:
                break

        #Determine the steering:
        if count >0:
            target = pos_sum * (1/count)
            desired = (target - boid.pos).to_len(boid.max_vel)
            steer = desired - boid.vel
            return steer.limit(boid.max_force)*weight
        else:
            return pos_sum# will be Vector([0,0])
    @staticmethod
    def Separate(boid,boid_list,distance_map,weight,ranger,max_neighbor=10):
        '''
        :description: calculates the weighted separation force for a given boid based upon the boid list and distance map etc...
        :param boid: the current boid for which the calculation is being done....
        :param boid_list: a list containing all of the boids
        :param distance_map: a dictionary of key=index, value=distance for every boid in the boid_list
        :param weight:  the weight for this rule
        :param range:   the range for which this rule applies
        :return: the force vector...
        '''
        # store
        pos_sum = Vector.zeros(dim=boid.pos.shape())
        count = 0

        #iterate over the distances in the map...
        for key in distance_map:
            # Note for future use: if cohesion is a per swarm property that is to say that cohesion exists differently (relative varying ammounts)
            # or not at all between swarms in the latter case the algorithm could be sped up by partitioning the swarm space so it does not have t
            # to iterate over known partitions for which it does not interact....
            edge_dist = distance_map[key]
            # compare the edge distance to influence radius of the current boid
            if 0<edge_dist and edge_dist <= ranger:#  if the edge dist = 0 ==> it is itself so exclude....
                count += 1
                #calculate current separation distance direction and weight by edge distance
                pos_sum = pos_sum + ((boid.pos-boid_list[key].pos).normalized()* (1/edge_dist))
            if  bool(max_neighbor) and count >=max_neighbor:
                break

        #Now convert to force...
        if count>0:
            pos_sum = pos_sum * (1/count)#convert it to an average
            steer = (pos_sum.to_len(boid.max_vel)-boid.vel).limit(boid.max_force)# Steering
            return steer*weight
        else:
            return Vector.zeros(dim=boid.pos.shape())
    @staticmethod
    def  Align(boid,boid_list,distance_map,weight,ranger,max_neighbor=10):
        '''
        :description: calculates the weighted alignment force for a given boid based upon the boid list and distance map etc...
        :param boid: the current boid for which the calculation is being done....
        :param boid_list: a list containing all of the boids
        :param distance_map: a dictionary of key=index, value=distance for every boid in the boid_list
        :param weight:  the weight for this rule
        :param range:   the range for which this rule applies
        :return: the force vector...
        '''
        # store
        vel_sum = Vector.zeros(dim=boid.vel.shape())
        count = 0

        #iterate over the distances in the map...
        for key in distance_map:
            # Note for future use: if cohesion is a per swarm property that is to say that cohesion exists differently (relative varying ammounts)
            # or not at all between swarms in the latter case the algorithm could be sped up by partitioning the swarm space so it does not have t
            # to iterate over known partitions for which it does not interact....
            edge_dist = distance_map[key]
            # compare the edge distance to influence radius of the current boid
            if 0<edge_dist and edge_dist <= ranger:#  if the edge dist = 0 ==> it is itself so exclude....
                count += 1
                #calculate current separation distance direction and weight by edge distance
                vel_sum = vel_sum + boid_list[key].vel
            if  bool(max_neighbor) and count >=max_neighbor:
                break
        #Now convert to force...
        if count>0:
            vel_sum = vel_sum * (1/count)#convert it to an average
            steer = (vel_sum.to_len(boid.max_vel)+boid.vel).limit(boid.max_force)# Steering
            return steer*weight
        else:
            return Vector.zeros(dim=boid.pos.shape())
    #avoid boids in front view... turn...
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
            return (pos_0-pos_1).mag() if not squared else sum((pos_0-pos_1).apply(lambda x: x*x).components)
        else:
            #use the equation i derived for modular space euclidian distance   min(|x1-x2|,max_x - |x1-x2|)   same applies to y (note the order is not specific for x2-x1 or x1-x2 but it must be consistant

            #Compte the seperation of the components
            delta = (pos_0-pos_1).apply(abs)
            #compute distance from boundry
            mod_dist  = modular_space - delta
            #compute the minumum distance for each... ==> distance components
            dist_vect = Vector.comp(min,delta,mod_dist)
            #returns the distance between the two n dimensional points in modular space
            return dist_vect.mag() if not squared else sum(dist_vect.apply(lambda x:x*x).components)
    @staticmethod#@timeit
    #@deprecated(reason="This is n^2 time complexity... use calc_distance_mapV2 as it has a time complexity of 1/2 n^2 + 1/2 n")
    def calc_distance_map(list_of_positions, modular_space=False,squared=False):#Using timeit on this determined that this was the slowest part of thecode ...
        '''
        :description: uses a caching based approach to calculate a dictionaries of dictionaries containing the position distances:
                          A   to    B         dist
                      {index:{ otherindex: distance,....},....}
        :param list_of_positions: a list of position Vectors [Vector,Vector,...,Vector]
        :modular_space: see euclidian_distance docstring
        squared: see euclidian_distance docstring
        :return: (see description)
        '''
        cache={}# really only gets a slim asyntotic advantage...
        for i in range(0,len(list_of_positions)):
            #Determine if the current entry is empty if so put a new array else preserve it
            if i not in cache:
                cache[i] = {}
            #now we need to go through all the positions
            for j in range(0,len(list_of_positions)):
                #if a positon edge is not already in the cache we need to compute it
                if j not in cache[i]:
                    #Run the 'intensive' calculation
                    dist = BoidUtils.euclidian_distance(list_of_positions[i],list_of_positions[j],modular_space=modular_space,squared=squared)
                    #Now we add it to the two locations it will be A-B and B-A    --> cache[i][j] and cache[j][i]
                    cache[i][j] = dist# we know cache[i] exists so add it to that dictionary
                    #we are however not sure if cache[j] exists yet and if it does not we need to init it first (rather if there is a dictionary at key j)
                    if j not in cache:
                        cache[j]={}
                    #now we can add the distance...
                    cache[j][i] = dist
                else:
                    pass # it was added to both dictionaries above...
        #(str(c)+" " + str(len(list_of_positions)*len(list_of_positions)))
        return cache
    @staticmethod#@timeit
    def calc_distance_mapV2(list_of_positions, modular_space=False,squared=False):
        #Using timeit on this determined that this was the slowest part of thecode ... V2 is the realization that
        #the first distance map while efficient ... was still technically O(n^2) not the more desired O([1/2]n^2 + [1/2]n)
        #this code fixes that...
        '''
        :description: uses a caching based approach to calculate a dictionaries of dictionaries containing the position distances:
                          A   to    B         dist
                      {index:{ otherindex: distance,....},....}
        :param list_of_positions: a list of position Vectors [Vector,Vector,...,Vector]
        :modular_space: see euclidian_distance docstring (note this may slow down by up to a factor of 4!!!
        squared: see euclidian_distance docstring
        :return: (see description)
        '''
        cache={}# really only gets a slim asyntotic advantage...
        for i in range(0,len(list_of_positions)):
            #Determine if the current entry is empty if so put a new array else preserve it
            if i not in cache:# at a certian point i takes on the value that j was ... so to prevent overriding
                cache[i] = {}
            #now we need to go through all the positions
            for j in range(len(list_of_positions)-1,i,-1):
                dist = BoidUtils.euclidian_distance(list_of_positions[i],list_of_positions[j],modular_space=modular_space,squared=squared)
                #Now we add it to the two locations it will be A-B and B-A    --> cache[i][j] and cache[j][i]
                cache[i][j] = dist# we know cache[i] exists so add it to that dictionary
                #we are however not sure if cache[j] exists yet and if it does not we need to init it first (rather if there is a dictionary at key j)
                if j not in cache:#we need to have a dictionary there to put a value... make sure not to overwrite it...
                    cache[j]={}
                #now we can add the distance...
                cache[j][i] = dist
        return cache