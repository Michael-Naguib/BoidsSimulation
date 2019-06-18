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

#Stores all the associated information with a boid
class Boid():
    def __init__(self,x_0,y_0,color=(0,0,0)):
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
        self.influence_radius=5#        the boid considers other boids within this distance from itself... (influence type depends on boid type)
        #Asthetics (what the Boid looks like)
        self.trail_color=(196,196,196)# specifies the color of the boids path trail (light grey by default)
        self.trail_visible=False#       states whether the trail is shown
        self.trail_life=3000#           number of milliseconds before a stroke of the trail fades
        self.color=color#               the color of the boid see __init__ parameter color descriptio

        #========= ADVANCED (expiramental) BOID INFO
        #Descriptive information
        self.id= uuid.uuid4()#          a unique id for the boid
        self.name=None#                 an optional name for the boid
        self.type=None#                 specifies a type of boid (i.e there may be different interacting swarms)
        #Extra Information:
        self.weight=1#                  an idea i had --> some boids have more importance when calculating cohesion for center of mass see document description comment (1=identity and no effect if all are 1)
        #(planned) Statistical Information: idea=keep track of # of close calls with predators ... use genetic algorithm for update rule params
