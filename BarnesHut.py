'''
# Boid
- Code By Michael Sherif Naguib
- license: MIT
- Date: 7/1/19
- @University of Tulsa
## Description:
- An implementation of the Barnes Hut Algorithm in basic python 3;
- Desired Features: Numpy Compatable, FAST!!! , Hyper-Param THETA setting, etc...
- NOTE! ... yes I have been inconsistent on handling vectors but sometimes a regular implementation is better than numpy...
'''
#imports
import numpy as np
from deprecated import deprecated# I❤🐍
import math
import copy

class BarnesHut():
    def __init__(self,all_points,all_masses,bounds,dim=3):
        '''
        :description:
        :param all_points:
        :param all_masses:
        :param bounds: sets the upper and lower coordinate bounds....
                    nd ==> [(x0,y0,...),(delta_x,delta_y,...)]

                    so for example a 30x100x400 rectangular prism centered at (0,15,10) would be
                    [(0,15,10),(30,100,400)]
        :param dim: sets the dimensions the algorithm computes in ...
        '''
        #check mass list len and point list len are equiv
        assert(len(all_masses)==len(all_points))
        #Construct a tree ... for 2d case ==> Quad Tree
        tree_root = Ntree(int(math.pow(2,dim)))# ex. dim==2 ==> we need an quad tree ... dim==3 ==> we need an oct tree ==> 2^dim
        #Recursivly divide up the space...

        pass
    @staticmethod
    def calc_COM(mass_list,positions_list):#Center Of Mass Calculation (non updatable)
        '''
        :description: Calculates the Center Of mass for a given set of points and masses
        :param mass_list: a list containing values for each mass corresponding to the positions in the position_list
        :param positions_list: a list of numpy arrays (nd.array) containing the positions for each mass
        :return: a numpy array vector representing the center of mass position vector
        '''

        #check: if they are not the same size ... then something went wrong --> need a mass for every position
        assert(len(mass_list) == len(positions_list))
        #sum of the masses
        mass_total = sum(mass_list)
        #check: if the mass is zero then there can be no Center Of Mass...
        assert(mass_total!=0)
        #init: will hold the weighted position vect
        weighted_position= np.zeros(positions_list[0].shape)
        #iter over
        for i in range(0,len(mass_list)):
            weighted_position = np.add(weighted_position,(positions_list[i]*mass_list[i]))# Vector + (Vector * scalar)   (broadcast)
        #return COM ==> (m1P1 + m2P2 + ...)* (1/m_tot)=  weighted_sum * (1/m_tot)
        return weighted_position * (1/mass_total)
    @staticmethod#Technically this is useless... just keep track of COM Position, and its total mass and you can use regular calc_COM
    @deprecated(reason="Technically this is useless... just keep track of COM Position, and its total mass and you can use regular calc_COM")
    def calc_COM_running(current_mass_sum,current_com_vect,additional_masses_list,additional_positions_list):
        '''
        :description: Preforms a running calculation of Center Of Mass ... given a current vector representing the
                      center of mass, and a list of new masses & a list of new positions, recalculate the center of mass
                      efficiently...
        :param current_mass_sum: a scalar that is the value of the total mass for the current center of mass position vector
        :param current_com_vect: a vector representing the current center of mass
        :param additional_masses_list: a list of new masses corresponding to the position vectors
        :param additional_positions_list:
        :return: a numpy array vector representing the center of mass position vector
        '''
        #add the two separate masses
        additional_masses_sum = sum(additional_masses_list)
        total_mass = current_mass_sum + additional_masses_sum

        #check: mass sum still should not be zero...
        assert(total_mass != 0)

        #Now calculate the weighted sum for the additional masses NOTE! code is similar to calc_COM
        #init: will hold the weighted position vect
        weighted_position= np.zeros(additional_positions_list[0].shape)
        #iter over
        for i in range(0,len(additional_masses_list)):
            weighted_position = np.add(weighted_position,(additional_positions_list[i]*additional_masses_list[i]))# Vector + (Vector * scalar)   (broadcast)

        #add the weighted sums and multiply by the combined total mass: (note to get the first weighted sum ... multiply the current com by the current mass)
        return np.add((current_com_vect*current_mass_sum),weighted_position) *(1/total_mass)
    @staticmethod
    def get_midpoint(point_0,point_1):
        '''
        :description: returns the midpoint position vector of the two position points (
        :param point_0: a starting position (as a numpy array)
        :param point_1: an ending position (as a numpy array)
        :return: midpoint position (directed)
        '''
        #midpoint = (p1-p0)*1/2
        return np.subtract(point_1,point_0)*1/2
    @staticmethod
    def sub_divide_box_nd(bounds):
        '''
        :param bounds:[(x0,y0,...),(delta_x,delta_y,...)] etc... delta_ is the displacement from that _0 etc...
        :return: returns all the bounds of the new n dimensional boxes after dividing... ex 2d ==> 4 boxes, 3d ==> 8 boxes
        '''
        #create a list containing the two halved sub parts for each box ... ex. [{(x_0,Δx/2),(Δx/2,Δx)}, ...., etc...]
        sub_parts = [
            [
                [bounds[0][i],bounds[0][i]+bounds[1][i]/2],
                [bounds[0][i]+bounds[1][i]/2,bounds[0][i]+bounds[1][i]]
            ] for i in range(0,len(bounds[0]))]
        #this is kind of the same as creating a truth table where x,y,z vectors are the positions and vect x_1 or 0 ... for x,y,z are the values
        # this ensures that each box bounding is created
        #create the combinations:init storage
        combo_list = [ None for i in range(int(math.pow(2,len(sub_parts))))]
        for i in range(0,len(combo_list)):
            combo_list[i] = [None for j in range(0,len(sub_parts))]
        iter_num = math.pow(2,len(sub_parts)-1)
        flip=True
        for i in range(0,len(sub_parts)):
            for j in range(0,len(combo_list)):
                if j%iter_num ==0:
                    flip = not flip
                if flip:
                    combo_list[j][i] = copy.deepcopy(sub_parts[i][0])#deep copy bc if it is referenced twice then ops will mod eachother
                else:
                    combo_list[j][i] = copy.deepcopy(sub_parts[i][1])
            iter_num = iter_num/2

        #now convert it into the familiar format that this function accepted...
        #this justmeans subtracting fro vect y = (y0,y1) ==> y1 = y1-y0 for each
        for i in range(0,len(combo_list)):
            for j in range(0,len(combo_list[i])):
                combo_list[i][j][1] = combo_list[i][j][1] - combo_list[i][j][0]
        return combo_list
    @staticmethod
    def point_in_box(point,box):
        '''
        :description: determines if a point is inside the box...
        :param point: a list of values...
        :param box: [(x0,y0,...),(delta_x,delta_y,...)] etc...
        :return: True if point in box false otherwise
        '''

        #iter
        for i in range(0,len(box[0])): #for each axis
            if not (box[0][i]<point[i] and point[i]< box[0][i]+box[1][i]): # if the cordinate is not inside the bound
                return False# return false and exit early
        #else we never returned false so true!
        return True


# Use for Binary, Trinry, Quad, ... Oct trees
class Ntree():
    def __init__(self,n,value=None):
        #using a linked structure... stored in a list
        self.children = [None for i in range(0,n)]
        self.parent=None
        self.value = value
    #sets the nth child tree not value (index by 0)
    def _set_Qn(self,tree_struct,n):
        self.children[n] = tree_struct
        if tree_struct != None:
            tree_struct.parent = self
    #get the nth children (quadrent Qn ... n=0,1,2,3,4) INDEX BY 0
    def get_child(self,n):
        return self.children[n]
    #gets the value of the curent node
    def get_value(self):
        return self.value

if __name__ == "__main__":
    #print(BarnesHut.sub_divide_box_nd([[0,6],[10,18]]))# Test: correct output [[[5.0, 5.0], [15.0, 9.0]], [[5.0, 5.0], [6, 9.0]], [[0, 5.0], [15.0, 9.0]], [[0, 5.0], [6, 9.0]]]
    pass






