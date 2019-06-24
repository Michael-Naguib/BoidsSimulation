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

#Stores all the associated information with a boid
class Boid():
    def __init__(self,x_0,y_0,color='black'):
        '''
        :description: Creates a Boid (https://www.red3d.com/cwr/boids/)                     (side note! i just figured out pycharm can add in docstrings! first time using these... fairly useful!)
        :param x_0: is the initial x coordinate of the boid
        :param y_0: is the initial y coordinate of the boid
        :param color: defaults to black is a tuple of three integers specifying an rgb color of the boid's drawing example (255,255,255)
        '''
        #======= BASIC BOID SIMULATION INFO
        #Position Information:
        self.x=x_0#                     is the initial x coordinate of the boid
        self.y=y_0#                     is the initial y coordinate of the boid
        #Velocity Information:
        self.dxdt=0#                    the velocity of the boid in the x direction
        self.dydt=0#                    the velocity of the boid in the y direction
        #Influence Range:
        self.influence_radius=20#        the boid considers other boids within this distance from itself... (influence type depends on boid type)
        #Asthetics (what the Boid looks like)
        #self.trail_color='grey'# specifies the color of the boids path trail (light grey by default)
        #self.trail_visible=False#       states whether the trail is shown
        #self.trail_life=3000#           number of milliseconds before a stroke of the trail fades
        self.color=color#               the color of the boid see __init__ parameter color descriptio
        self.scale=1#            increases the size of the drawing of the boid's circle
        #========= ADVANCED (expiramental) BOID INFO
        #Descriptive information
        self.id= uuid.uuid4()#          a unique id for the boid
        self.name=None#                 an optional name for the boid
        self.type=None#                 specifies a type of boid (i.e there may be different interacting swarms)
        #Extra Information:
        self.weight=1#                  an idea i had --> some boids have more importance when calculating cohesion for center of mass see document description comment (1=identity and no effect if all are 1)
        #(planned) Statistical Information: idea=keep track of # of close calls with predators ... use genetic algorithm for update rule params
    def _draw(self,tkinter_canvas):
        #center the boid  WARNING! ==> Update position should be bounded ==> self.x-scale this bounds it by x,y =0 & x,y=max
        x0 = max(0,(self.x - self.scale))
        y0 = max(0,(self.y - self.scale))
        x1 = min(int(tkinter_canvas.cget('width')),(self.x + self.scale))
        y1 = min(int(tkinter_canvas.cget('height')),(self.y + self.scale))
        tkinter_canvas.create_oval((x0,y0,x1,y1),fill=self.color)
    def _update_position(self,canvas_width,canvas_height):
        '''
        :description:  uses the velocity and the modular space constraints (screen height and width) to compute the next
                       position of the boid; NOTE! Velocity should manually be updated FIRST!
        '''
        self.x = (self.dxdt +self.x) % canvas_width
        self.y = (self.dydt + self.y) % canvas_height
    def __str__(self):
        xstr = (str(self.x)+"     ")[0:5]#truncates with 5 spaces if smaller
        ystr = (str(self.y) + "     ")[0:5]  # truncates with 5 spaces if smaller
        dydt = (str(self.dydt) + "     ")[0:5]
        dxdt = (str(self.dxdt) + "     ")[0:5]
        return "X:{0} Y:{1} dydt:{2} dxdt:{3} id:{4}".format(xstr,ystr,dydt,dxdt,str(self.id))