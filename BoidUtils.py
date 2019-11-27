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
import random
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
    def limitVect(vect,mag):
        vmag = np.linalg.norm(vect)
        if vmag>mag:
            return (mag/vmag)*vect
        else:
            return vect
    @staticmethod
    def inViewArc(boid,pos,arcAngle):
        '''
        :description: determines if a position is withine a certian arc of a boid ...
                      if the boid's velocity is considered the reference vector ... the position
                      is considered inside the arc if it is in +-arcAngle (i.e the left half or the right half)
        :param boid:        boid instance
        :param pos:         a position vector
        :param arcAngle:    and angle 0,2PI
        :return: Boolean
        '''
        # Get the angle between the the two vectors ...
        # use a dot product ... abs(a.b) = ||a|| ||b|| cos(theta)
        # compute a.b and mag a mag b use cos inverse...  be sure to add in a correction
        # factor for determining which quadrent
        adp = abs(sum([boid.vel[i]*pos[i] for i in range(len(pos))]))
        mp = np.linalg.norm(pos)*np.linalg.norm(boid.vel)
        if mp ==0:
            return False
        #Otherwise
        theta = math.acos(adp/mp)
        return theta <= arcAngle


    @staticmethod
    def cohereForce(boid,boids,neighbors,distances,distance,dim=3,max_vel=None,max_accel=None,arcView=None):
        '''
        :description:       Calculates the coherence force update for the boids sim for a particular boid
        :param boids:       a list of all the boids
        :param neighbors:   a list of indicies of the neighbors of the boid (implicit) we are computing for
        :param distances:   the distances coresponding to that boid to the other nearest boids...
        :param distance:    the max distance for which this rule is valid
        :param max_vel:     maximum velocity
        :param max_accel:   maximum acceleration
        :param dim:         the dimension for the vectors...
        :return:            vector representing the force update
        '''

        # Pos_Sum will be a weighted sum of positions used to calculate the center of mass
        pos_sum = np.zeros(dim)
        mass_sum=0
        count = 0
        # For every Neighbor:
        for indx in range(len(neighbors)):
            # Check to make sure it meets the distance requirements
            if distances[indx]> distance and BoidUtils.inViewArc(boid,boids[indx].pos,arcView):
                continue # Goto the next
            # otherwise: add the position weighted by the mass to the sum
            mass = boids[neighbors[indx]].mass
            mass_sum+=mass
            pos_sum = np.add(pos_sum,boids[neighbors[indx]].pos*(1/mass))
            count+=1
        # If there were no force updates return 0 vect
        if count==0:
            return pos_sum
        # otherwise: compute the cohere force...
        target = np.subtract(pos_sum*(1/mass_sum),boid.pos)
        # Normalize the target and set it to the magnitue to max velocity
        target_mag = np.linalg.norm(target)
        # If the magnutude of target is zero then do nothing...
        target = target*(max_vel/target_mag) if target_mag!=0 else target
        steer = np.subtract(target,boid.vel)
        # Limit the acceleration
        steer = BoidUtils.limitVect(steer,max_accel)
        return steer
    @staticmethod
    def seperateForce(boid,boids,neighbors,distances,distance,dim=3,max_vel=None,max_accel=None,arcView=None):
        '''
        :description:       Calculates the seperation force update for the boids sim for a particular boid
        :param boids:       a list of all the boids
        :param neighbors:   a list of indicies of the neighbors of the boid (implicit) we are computing for
        :param distances:   the distances coresponding to that boid to the other nearest boids...
        :param distance:    the max distance for which this rule is valid
        :param max_vel:     maximum velocity
        :param max_accel:   maximum acceleration
        :param dim:         the dimension for the vectors...
        :return:            vector representing the force update
        '''
        pos_sum = np.zeros(dim)
        count = 0
        mass_sum=0
        # For every Neighbor:
        for indx in range(len(neighbors)):
            # Check to make sure it meets the distance requirements
            if distances[indx]> distance and BoidUtils.inViewArc(boid,boids[indx].pos,arcView):
                continue# Goto the next
            # otherwise
            # calculate current separation distance direction and weight by edge distance
            edge_dist = distances[indx]
            sep_norm = np.subtract(boid.pos,boids[neighbors[indx]].pos)* (1/(edge_dist if edge_dist!=0 else random.random()))# Catch divide by zero errors by inroducing a small random value
            # since the two boids overlap exactly ... introduce some small uncertianty in the distance ... interesting ... this makes sense from a practical programatic perspective...
            # uncertainty in the location of an item at small scales makes sense ... Heisenburg uncertianty?
            mass = boids[neighbors[indx]].mass
            pos_sum = np.add(pos_sum,sep_norm*mass)
            count+=1
            mass_sum+=mass
        # If there were no force updates return 0 vect
        if count==0:
            return pos_sum
        # otherwise
        target = pos_sum*(1/mass_sum)
        target_mag = np.linalg.norm(target)
        target = target*(max_vel/target_mag) if target_mag!=0 else target
        steer = np.subtract(target,boid.vel)
        steer = BoidUtils.limitVect(steer, max_accel)
        return steer
    @staticmethod
    def alignForce(boid,boids,neighbors,distances,distance,dim=3,max_vel=None,max_accel=None,arcView=None):
        '''
        :description:       Calculates the alignment force update for the boids sim for a particular boid
        :param boids:       a list of all the boids
        :param neighbors:   a list of indicies of the neighbors of the boid (implicit) we are computing for
        :param distances:   the distances coresponding to that boid to the other nearest boids...
        :param distance:    the max distance for which this rule is valid
        :param max_vel:     maximum velocity
        :param max_accel:   maximum acceleration
        :param dim:         the dimension for the vectors...
        :return:            vector representing the force update
        '''
        vel_sum = np.zeros(dim)
        mass_sum=0
        count = 0
        # For every Neighbor:
        for indx in range(len(neighbors)):
            # Check to make sure it meets the distance requirements
            if distances[indx]> distance and BoidUtils.inViewArc(boid,boids[indx].pos,arcView):
                continue# Goto the next
            # otherwise
            mass = boids[neighbors[indx]].mass
            mass_sum+=mass
            vel_sum = np.add(vel_sum,boids[indx].vel*mass)
            count+=1
        # If there were no force updates return 0 vect
        if count==0:
            return vel_sum
        # otherwise
        target = vel_sum*(1/(mass_sum))
        target_mag = np.linalg.norm(target)
        target = target * (max_vel / target_mag) if target_mag != 0 else target
        steer = np.subtract(target,boid.vel)
        steer = BoidUtils.limitVect(steer, max_accel)
        return steer

