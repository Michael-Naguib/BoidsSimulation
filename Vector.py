'''
# Boids Simulation
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/26/19
- @University of Tulsa
## Description:
- This is code to create a new vectors...
'''
#imports
import math
import copy
import random
import numpy as np
#Settings
random.seed(a=31415)

class Vector():#TODO: refine implementation so not so many new vectors are constantly created!!
    def __init__(self,components_list):
        '''
        :description: constructor for the vector class
        :param components_list: a list containing the componenets of the vector ex. for x=2 y=4 ==> pass [2,4] .. works with n components
        '''
        self.components = components_list

    def __add__(self, other):
        '''
        :description: adds a vector to either a scalar or another vector
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if  isinstance(other,Vector):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (other.components[i]+ self.components[i]) for i in range(0,len(other.components))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(other + self.components[i]) for i in range(0, len(self.components))]
            return Vector(newComponents_list)
    def __sub__(self, other):
        '''
        :description: subtracts from a vector either elements of another vector or a scalar
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if  isinstance(other,Vector):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (self.components[i]-other.components[i]) for i in range(0,len(other.components))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(self.components[i]-other) for i in range(0, len(self.components))]
            return Vector(newComponents_list)
    def __mul__(self, other):
        '''
        :description: returns the hadamard (schur) product of the two vectors or simply distributes a scalar across all components
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if  isinstance(other,Vector):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (self.components[i]*other.components[i]) for i in range(0,len(other.components))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(self.components[i]*other) for i in range(0, len(self.components))]
            return Vector(newComponents_list)
    def __mod__(self, other):
        '''
        :description: returns elementwise modular arithmatic V1=(a,b) V2=(c,d) ==> (a%c,b%d)  or if the
        seccond parameter is a scalar then it is done V1=(a,b)   V1 % c == >  (a%c,b%c)
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if  isinstance(other,Vector):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (self.components[i]%other.components[i]) for i in range(0,len(other.components))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(self.components[i]%other) for i in range(0, len(self.components))]
            return Vector(newComponents_list)
    def __str__(self):
        '''
        :description: returns a string representation of the vector
        :return: a string representation of the vector
        '''
        return str(self.components)
    def __len__(self):
        #DEPRICATED!!!!!! --> for some reason python len(instance) is not the same as instance.__len__()
        # the latter can return a float ... the former returns a error...
        '''
        :description: returns vector length
        :return: returns the length of the vector
        '''

        return #int(math.sqrt(sum([math.pow(self.components[i],2) for i in range(0,len(self.components))])))
    def mag(self):
        '''
        :description: returns vector length
        :return: returns the length of the vector
        '''

        return int(math.sqrt(sum([math.pow(self.components[i],2) for i in range(0,len(self.components))])))
    def __abs__(self):
        '''
        :description: returns the elementwise absolute value of each vector ...
        :return: vector of absolute values
        '''
        return self.apply(abs)
    def __gt__(self, other):
        '''
        :description: compares either two vectors element wise or a scalar to each component...
                      if even one component does not meet the criteria it will return false
                      ex. (42,21,0) > 0 ==> (true,true,false) ==> the function returns false
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if  isinstance(other,Vector):#vector case
            assert (len(self.components) == len(other.components))
            #check each component
            for i in range(0,len(self.components)):
                if not self.components[i]>other.components[i]:
                    return False
            return True
        else:#scalar case
            # check each component
            for i in range(0, len(self.components)):
                if not self.components[i] > other:
                    return False
            return True
    def __lt__(self, other):
        '''
        :description: compares either two vectors element wise or a scalar to each component...
                      if even one component does not meet the criteria it will return false
                      ex. (42,21,-1) < 0 ==> (false,false,true) ==> the function returns false
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if  isinstance(other,Vector):#vector case
            assert (len(self.components) == len(other.components))
            #check each component
            for i in range(0,len(self.components)):
                if not self.components[i]<other.components[i]:
                    return False
            return True
        else:#scalar case
            # check each component
            for i in range(0, len(self.components)):
                if not self.components[i] < other:
                    return False
            return True
    def __copy__(self):
        '''
        :description: returns an exact copy of itself (a new object)
        :return: read description
        '''
        return copy.deepcopy(self)
    def __bool__(self):
        return True
    def to_len(self,new_len):
        '''
        :description: creates a new vector with the orientation of the current and the length specified
        :param new_len: the new length for the vector
        :return: a new vector...
        '''
        return self.normalized()*new_len
    def normalized(self):
        '''
        :description: returns a normalized vector
        :return:
        '''
        #multiply the current vector by 1/len --> returns a new vector
        mag = self.mag()
        return (self)*(1/mag if mag!=0 else 0)
    def limit(self,max_val):
        '''
        :description: limits the components of one vector according to another --> ensures direction is maintained
                      but limits the magnitude
        :param max_val: a scalar representing the maximum mag
        :return: a new vector limited as described aboce
        '''
        mag = self.mag()
        if mag >= max_val:
            return self.normalized()*max_val
        else:
            return copy.deepcopy(self)
    def apply(self,func):
        '''
        :param func: a function to be applied to each component
        :return: returns a new vector where that function is applied to each component in the vector
        '''
        components = [func(self.components[i]) for i in range(0,len(self.components))]
        return Vector(components)
    @staticmethod
    def rand(max_range=1,dim=3):
        '''
        :description: creates a vector with random components (within a 0-maxrange) and for dim number of components
        :param max_range: the maximum value a component can be
        :param dim: the number of components
        :return: a new random vector
        '''
        components = [max_range*random.random() for i in range(0,dim)]
        return Vector(components)
    @property
    def x(self):
        '''
        :description: a little bit of syntatic sugar for accessing variables
        :return: the x component
        '''
        return self.components[0]
    @property
    def y(self):
        '''
        :description: a little bit of syntatic sugar for accessing variables
        :return: the y component
        '''
        return self.components[1]
    @property
    def z(self):
        '''
        :description: a little bit of syntatic sugar for accessing variables
        :return: the y component
        '''
        return self.components[2]
    @staticmethod
    def comp(func,vect_0,vect_1):
        '''
        :description: compare; calls the function on both parameters in vect0,vect1 order so as to compare them ... the
                      the returned vectors components are the results of calling the function on each pair of
                      components
        :param vect_0: a vector
        :param vect_1: a vector
        :return: returns a new vector instance
        '''
        assert(len(vect_0.components)==len(vect_1.components))
        components =  [func(vect_0.components[i],vect_1.components[i]) for i in range(0,len(vect_0.components))]
        return Vector(components)

