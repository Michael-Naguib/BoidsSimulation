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

class Vector():
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
        if type(other)== type("Vector"):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (other.components[i]+ self.components[i]) for i in range(0,len(other))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(other + self.components[i]) for i in range(0, len(other))]
            return Vector(newComponents_list)
    def __sub__(self, other):
        '''
        :description: subtracts from a vector either elements of another vector or a scalar
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if type(other)== type("Vector"):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (self.components[i]-other.components[i]) for i in range(0,len(other))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(self.components[i]-other) for i in range(0, len(other))]
            return Vector(newComponents_list)
    def __mul__(self, other):
        '''
        :description: returns the hadamard (schur) product of the two vectors or simply distributes a scalar across all components
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if type(other)== type("Vector"):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (self.components[i]*other.components[i]) for i in range(0,len(other))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(self.components[i]*other) for i in range(0, len(other))]
            return Vector(newComponents_list)
    def __mod__(self, other):
        '''
        :description: returns elementwise modular arithmatic V1=(a,b) V2=(c,d) ==> (a%c,b%d)  or if the
        seccond parameter is a scalar then it is done V1=(a,b)   V1 % c == >  (a%c,b%c)
        :param other: can be a vector or a scalar ... if it is a vector the # of items in each must match
        :return: returns a new vector instance with operation preformed
        '''
        #Is it a vector and of the same size
        if type(other)== type("Vector"):
            assert (len(self.components) == len(other.components))
            #iterate summing elementwise
            newComponents_list = [ (self.components[i]%other.components[i]) for i in range(0,len(other))]
            return Vector(newComponents_list)
        else:
            #assume its a scalar --> add the scalar to every element
            newComponents_list = [(self.components[i]%other) for i in range(0, len(other))]
            return Vector(newComponents_list)
    def __str__(self):
        '''
        :description: returns a string representation of the vector
        :return: a string representation of the vector
        '''
        return str(self.components)
    def __len__(self):
        '''
        :description: returns vector length
        :return: returns the length of the vector
        '''
        return math.sqrt(sum([math.pow(self.components[i],2) for i in range(0,len(self.components))]))
    def normalized(self):
        '''
        :description: returns a normalized vector
        :return:
        '''
        #multiply the current vector by 1/len --> returns a new vector
        return (self)*(1/len(self))
    def apply(self,func):
        '''
        :param func: a function to be applied to each component
        :return: returns a new vector where that function is applied to each component in the vector
        '''
        components = [func(self.components[i]) for i in range(0,len(self.components))]
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
        components =  [func(vect_0.components[i],vect_1.components[i]) for i in range(0,len(vect0.components))]

