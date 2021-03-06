'''
# Boid
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
- Description: this file holds the code for the basic structure of a boid... the info it contains
'''
#imports
import numpy as np

# The basic structure for storing information about a boid
class Boid():
    def __init__(self,pos,vel,accel,mass=1.0,color=(0,0,0)):
        '''
        :description: Creates a Boid (https://www.red3d.com/cwr/boids/)
                      (side note! I just figured out pycharm can add in docstrings! first time using these... fairly useful!)
        :param pos:     is the initial position of the boid
        :param vel:     is the initial velocity of the boid
        :param accel:   is the initial acceleration of the boid
        :param mass:    is the initial mass of the boid; default=1 ==> no effect
        :param color:   is a tuple of three integers specifying an rgb color of the boid's drawing example
                        (255,255,255); default = black
        '''
        #======= BASIC BOID SIMULATION INFO
        # Position, Velocity, Acceleration, & mass Information:
        self.pos = np.array(pos)
        self.vel = np.array(vel)
        self.accel = np.array(accel)
        self.mass = mass

        # Stores the neighbor indicies and their coresponding distances...
        self.neighbors = None
        self.distances = None

        # Display Settings
        self.color=color # The color of the boid see __init__ parameter color description

        #========= BOID INFO
        # Descriptive information
        self.type=None  # specifies a type of boid (i.e there may be different interacting swarms)
    def __str__(self):
        '''
        :description: returns a string representation of the boid
        :return: str representation of the boid
        '''
        pos = (str(self.pos) + "            ")[0:13]# limits to 13 characters...
        vel = (str(self.vel)+ "            ")[0:13]
        accel = (str(self.accel)+ "            ")[0:13]
        return "Pos:{0}, Vel:{1}, Accel:{2}, Type:{3}, Color:{4}".format(pos,vel,accel,self.type,self.color)
