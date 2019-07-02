'''
# BoidSwarm
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
## Description:
- Modeling a Boids Flock (swarm) according to Separation Alignment & Cohesion and Avoidance Rules
  see: https://www.red3d.com/cwr/boids/
- The code needed a revision as it ran too slow: this is probably due the the large quantity of O(n^2) algorithms
  this could be improved by changing the simulation ...
- Also admittedly the code was horribly complex --> I have opted to go for basics and then after mastering that, add on
  functionality
'''

#imports
from Vector import Vector
from BoidUtils import BoidUtils
from Boid import Boid

#Helps create and run a boid swarm

class BoidSwarm():
    #========= SETUP
    #=== internal setup for this class do not modify
    def __init__(self,quantity,canvas_bounds):
        '''
        :description: internal class initialization for the swarm
        :param quantity: the desired number of boids to be created...
        :param canvas_bounds:  is a vector containing the dimensions of the field

        usage:
        bs = BoidSwarm(quantity=100,canvas_bounds=Vector([1920,1080]))#width x height
        bs.update_boid_positions()
        bs.draw_swarm(tkinter_canvas)

        '''
        #=== Update Rules & Boid Storage
        rv = Vector.rand# random vector generator... has config settings see Vector class
        d = min(canvas_bounds.x,canvas_bounds.y,canvas_bounds.z)# make sure the random ness is contained in the space...
        #here wer are init a random posittion,velocity,acceleration
        self.boid_list = [Boid(rv(max_range=d),rv(max_range=32),rv(max_range=3)) for i in range(quantity)]
        self.canvas_bounds = canvas_bounds
        #Multiprocessing Speedup settings
    #========= RUN
    #=== essentially advances the next time step
    def update_boid_positions(self):
        '''
        :description: updates all of the Accelerations, Velocities and Positions of the boids in the boid list according to the
                      list of present update rules and settings configured in each boid
        '''
        #calculate: The distance information  not using squared optimization because you would have to take the sqrt later on
        #           which defeats the purpose ... easier to do it here
        positions_list = [boid.pos for boid in self.boid_list]
        # O(n^2) so slow!!!
        distance_maps= BoidUtils.calc_distance_map(positions_list,modular_space=self.canvas_bounds,squared=False)
        #For each boid have it update its position using its distance map as well as a ref to the boid list and canvas bounds
        for i in range(0,len(self.boid_list)):
            self.boid_list[i].update_position(self.boid_list,distance_maps[i],self.canvas_bounds)
    #=== draw the boid swarm
    def draw_swarm(self,tkinter_canvas):
        for boid in self.boid_list:
            boid.draw(tkinter_canvas)
    #========= Utility Methods
    #=== get a string representation of a part of the swarm
    def __str__(self,quant=1):
        '''
        :description: print a limited quantity of the boids ... always takes from the front of the list
        :param quant: the desired quantity
        :return: string representation
        '''
        out=""
        for i in range(0,min(quant,len(self.boid_list))):
            out = out+ str(self.boid_list[i]) + "\n"
    #=== adds a boid at the given Vector
    def add_boid_at_pos(self,pos_vector):
        '''
        :description: adds a boid at the specified position set with a random velocity and acceleration
        :param pos_vector: a Vector containing the position of a new boid
        '''
        rv = Vector.rand  # random vector generator... has config settings see Vector class
        self.boid_list.add(Boid(pos_vector,rv(max_range=32),rv(max_range=3)))
    #=== deletes a boid: by id,index,random (may pass one kew word arg at a time)
    def delete_boid(self,id=False,index=False,random=False):
        '''
        :description: deletes a boid based on the supplied criteria
        :param id:   coresponding to the UUID property of each boid O(n) worst case
        :param index: the location of the boid in the boid list  O(n) worst case
        :param random: if set to True will delete a random boid O(n) worst case
        '''
        #handle error
        if (not id) and (not index) and not(random):
            raise BaseException("No arguments were passed")
        #Case: DELETE ID
        if bool(id):
            for i in range(0,len(self.boid_list)):
                if self.boid_list[i].id==id:
                    del self.boid_list[i]
                    break
        #Case: DELETE INDEX
        elif(bool(index)):
            for i in range(0,len(self.boid_list)):
                if i == index:
                    del self.boid_list[i]
                    break
        #Case: DELETE RANDOM
        elif(bool(random)):
            r = random.randint(0,len(self.boid_list)-1)
            del self.boid_list[r]






