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
        #Position, Velocity, & Acceleration Information:
        self.pos = pos
        self.vel = vel
        self.accel = accel
        self.max_vel = 9.8696#          sets the maximum velocity achievable... arbritrarily set to ~pi^2  here for example
        self.max_accel= 2.718#          sets the maximum acceleration...   arbritrarily set to ~e  here for example
        #Influence Range:
        self.rule_ranges=[]#             holds the distance for each rule...
        self.rule_weights=[]#            holds the 'weight' for each corresponding rule

        #Display Settings
        self.color=color#               the color of the boid see __init__ parameter color description
        self.scale=1#                   the scale to draw the boid! note this influences the screen bounds as it cant be drawn offscreen...
        #========= ADVANCED (expiramental) BOID INFO
        #Descriptive information
        self.id= uuid.uuid4()#          a unique id for the boid
        self.name=None#                 an optional name for the boid
        self.type=None#                 specifies a type of boid (i.e there may be different interacting swarms)
        self.mass=1

    def draw(self,tkinter_canvas):
        '''
        :description: takes a tkinter canvas and draws a representation of the boid to the canvas
        :param tkinter_canvas: a tkinter canvas object
        '''
        #Tkinter Dimensions Max Vector
        t_dim = Vector([int(tkinter_canvas.cget('width')),int(tkinter_canvas.cget('height'))])
        scale_vect=  Vector([self.scale,self.scale])
        #center the boid  WARNING! ==> Update position should be bounded ==> self.x-scale this bounds it by x,y =0 & x,y=max
        pos_0 = Vector.comp(max,Vector([0,0]),self.pos - scale_vect)#bottom left corner
        pos_f = Vector.comp(min,t_dim,self.pos + scale_vect)#top right corners
        #draw the boid ... note using syntatic sugar for position update
        tkinter_canvas.create_oval((pos_0.x,pos_0.y,pos_f.x,pos_f.y),fill=self.color)

    def update_position(self,max_dim_vect):
        '''
        :description:  uses the velocity and the modular space constraints (screen height and width) to compute the next
                       position of the boid; NOTE! Velocity should manually be updated FIRST!
        :param max_dim_vect: is a Vector specyfing the maximum value for x and y cordinates
        '''

        # F=MA ==> A = F/M  ==> Force F applied to Mass M induce Acceleration A

        #Determine the forces of all the other boids acting upon
        # accel |--> newaccel = min( (sigma all forces)/m ,  max accel )
        # vel   |--> newvel = min(vel + accel, max vel)
        # pos   |--> newpos = (pos + vel) mod (canvas bounds)

        self.x = (self.dxdt +self.x) % canvas_width
        self.y = (self.dydt + self.y) % canvas_height
    def __str__(self):
        '''
        :description: returns a string representation of the boid
        :return: str rep of boid
        '''
        xstr = (str(self.x)+"     ")[0:5]#truncates with 5 spaces if smaller
        ystr = (str(self.y) + "     ")[0:5]  # truncates with 5 spaces if smaller
        dydt = (str(self.dydt) + "     ")[0:5]
        dxdt = (str(self.dxdt) + "     ")[0:5]
        return "X:{0} Y:{1} dydt:{2} dxdt:{3} id:{4}".format(xstr,ystr,dydt,dxdt,str(self.id))