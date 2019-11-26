'''
# Boid
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
- Description: this file holds the code for the basic structure of a boid... the info it contains
'''

# The basic structure for storing information about a boid
class Boid():
    def __init__(self,pos,vel,acel,mass=1.0,color='black'):
        '''
        :description: Creates a Boid (https://www.red3d.com/cwr/boids/)
                      (side note! I just figured out pycharm can add in docstrings! first time using these... fairly useful!)
        :param pos:     is the initial position of the boid
        :param vel:     is the initial velocity of the boid
        :param acel:    is the initial velocity of the boid
        :param mass:    is the initial mass of the boid; default=1 ==> no effect
        :param color:   is a tuple of three integers specifying an rgb color of the boid's drawing example
                        (255,255,255); default = black
        '''
        #======= BASIC BOID SIMULATION INFO
        # Position, Velocity, Acceleration, & mass Information:
        self.pos = pos
        self.vel = vel
        self.acel = acel
        self.mass = mass

        # Display Settings
        self.color=color # The color of the boid see __init__ parameter color description
        self.scale=1     # The scale to draw the boid! note this influences the screen bounds as it cant be drawn offscreen...
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
        acel = (str(self.acel)+ "            ")[0:13]
        return "Pos:{0}, Vel:{1}, Acel:{2}, Type:{4}, Color:{5}".format(pos,vel,acel,self.type,self.color)
