'''
# Boid
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
## Description:
- This file holds the class for a Boid
'''

#imports
import uuid
import math
from Vector import Vector
from BoidUtils import BoidUtils
#Stores all the associated information with a boid
class Boid():
    def __init__(self,pos,vel,accel,color='black'):
        '''
        :description: Creates a Boid (https://www.red3d.com/cwr/boids/)                     (side note! i just figured out pycharm can add in docstrings! first time using these... fairly useful!)
        :param pos: is the initial position of the boid
        :param vel: is the initial velocity of the boid
        :param accel: is the initial acceleration of the boid (reset after every timestep)
        :param color: defaults to black is a tuple of three integers specifying an rgb color of the boid's drawing example (255,255,255)
        '''
        #======= BASIC BOID SIMULATION INFO
        #Position, Velocity, Acceleration, & mass Information:
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.max_vel = 9.8696#          sets the maximum velocity achievable... arbritrarily set to ~pi^2  here for example
        self.max_accel= 2.718*2#          sets the maximum acceleration...   arbritrarily set to ~e  here for example
        self.max_force = 0.05#          sets the maximum force that can be applied PER EACH RULE!
        self.mass = 0.02#                  sets the mass of the boid ... heavier boids require more force to move.... 1=no effect

        #Influence Range:
        self.rules=[BoidUtils.Align,BoidUtils.Cohere,BoidUtils.Separate]# Holds the references to the functions
        self.rule_ranges=[60,80,20]#            holds the distance for each rule...
        self.rule_weights=[9/18,1/18,7/18]#            holds the 'weight' for each corresponding rule

        #Display Settings
        self.color=color#               the color of the boid see __init__ parameter color description
        self.scale=1#                   the scale to draw the boid! note this influences the screen bounds as it cant be drawn offscreen...
        #========= ADVANCED (expiramental) BOID INFO
        #Descriptive information
        self.id= uuid.uuid4()#          a unique id for the boid
        self.name=None#                 an optional name for the boid
        self.type=None#                 specifies a type of boid (i.e there may be different interacting swarms)
    def draw(self,tkinter_canvas):
        '''
        :description: takes a tkinter canvas and draws a representation of the boid to the canvas
        :param tkinter_canvas: a tkinter canvas object
        '''
        #draw a circle whose center is at the current position of the boid and with a radius of the scale factor..
        bound_lower_left = self.pos + self.scale
        bound_upper_right = self.pos - self.scale
        d2Cord = (bound_lower_left.x,bound_lower_left.y,bound_upper_right.x,bound_upper_right.y)

        #draw the boid ... note using syntatic sugar for position update
        tkinter_canvas.create_oval(d2Cord,fill=self.color)
    def update_position(self,boids_list,distance_map,canvas_bounds):
        '''
        :description:  uses the velocity and the modular space constraints (screen height and width) to compute the next
                       position of the boid; NOTE! Velocity should manually be updated FIRST!
        :param distance_map: a dictionary containing the distances between this boid and all other boids
                             key: index value: distance
        :param canvas_bounds: is a Vector specifying the maximum value for x and y coordinates ex Vector([1920,1080])
        '''

        #Calculate the weight updates according to the boid rules...

        # F=MA ==> A = F/M  ==> Force F applied to Mass M induce Acceleration A
        #Determine the forces of all the other boids acting upon
        # fc = (sigma all forces)/m
        # accel |--> newaccel = fc.norm()*max_accel if abs(fc)>max accel for either component else fc
        # vel   |--> newvel = (vel+accel).norm()*max_vel if abs(vel+accel)>max_vel else  vel+accel
        # pos   |--> newpos = (pos + vel) mod (canvas bounds)      make sure to convert to ints ... for game grid
        # accel |--> 0

        #The Sum of the Forces from the Rules
        force_sum = Vector([0,0,0])
        for i in range(0,len(self.rules)):
            force_sum = force_sum + self.rules[i](self,boids_list,distance_map,self.rule_weights[i],self.rule_ranges[i])
        force_sum = force_sum*(1/self.mass)#factor in the mass of the boid...
        #Update the Acceleration according to the force and limit: (if either component is over)
        self.accel = force_sum.limit(self.max_accel)
        #Update the Velocity and Limit
        vel_update = self.vel+self.accel
        self.vel = vel_update.limit(self.max_vel)
        #Update the position and wrap the bounds...   add the current position and velocity then convert to int then mod canvas bounds
        self.pos = ((self.pos+self.vel).apply(int)) % canvas_bounds
        #finally reset the acceleration
        self.accel=Vector([0,0])
    def __str__(self):
        '''
        :description: returns a string representation of the boid
        :return: str rep of boid
        '''
        pos = (str(self.pos) + "            ")[0:13]# limits to 13 characters...
        vel = (str(self.vel)+ "            ")[0:13]
        accel_mag = (str(self.accel.mag())+ "            ")[0:13]
        return "Pos:{0} , Vel:{1} , Accel Mag:{2}, ID:{4}".format(pos,vel,accel_mag,str(self.id))