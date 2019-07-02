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
    def __init__(self,quantity,canvas_bounds):
        '''
        :description: internal class initialization for the swarm
        :param quantity: the desired number of boids to be created...
        :param canvas_bounds:  is a vector containing the dimensions of the field

        usage:
        bs = BoidSwarm(quantity=100,canvas_bounds=Vector([1920,1080]))#width x height
        bs.setup()# a custom function that specifies the types of boids created... based on self.initial_quantity
        LOOP
            bs.update_boid_positions()
            bs.draw_swarm(tkinter_canvas)

        '''
        self.canvas_bounds = canvas_bounds
        self.initial_quantity=quantity#DOES NOT UPDATE when new boids are added or deleted
        #Run the Boids setup...
    def setup(self):
        #=== Update Rules & Boid Storage
        rv = Vector.rand# random vector generator... has config settings see Vector class
        d = min(self.canvas_bounds.x,self.canvas_bounds.y,self.canvas_bounds.z)# make sure the random ness is contained in the space...
        #here wer are init a random posittion,velocity,acceleration
        self.boid_list = [Boid(rv(max_range=d),rv(max_range=32),rv(max_range=3)) for i in range(self.initial_quantity)]
    #========= RUN
    def update_boid_positions(self):
        '''
        :description: updates all of the Accelerations, Velocities and Positions of the boids in the boid list according to the
                      list of present update rules and settings configured in each boid (essentially advances the next time step)
        '''
        #calculate: The distance information  not using squared optimization because you would have to take the sqrt later on
        #           which defeats the purpose ... easier to do it here
        positions_list = [boid.pos for boid in self.boid_list]
        # O(n^2) so slow!!!
        distance_maps= BoidUtils.calc_distance_map(positions_list,modular_space=self.canvas_bounds,squared=False)
        #For each boid have it update its position using its distance map as well as a ref to the boid list and canvas bounds
        for i in range(0,len(self.boid_list)):
            self.boid_list[i].update_position(self.boid_list,distance_maps[i],self.canvas_bounds)
    def draw_swarm(self,tkinter_canvas):
        '''
        :description: draws the boid swarm to the canvas by calling the draw method for each boid (passing the canvas)
        :param tkinter_canvas:
        :return:
        '''
        for boid in self.boid_list:
            boid.draw(tkinter_canvas)
    #========= Utility Methods
    def __str__(self,quant=1):
        '''
        :description: print a limited quantity of the boids ... always takes from the front of the list
        :param quant: the desired quantity
        :return: string representation
        '''
        out=""
        for i in range(0,min(quant,len(self.boid_list))):
            out = out+ str(self.boid_list[i]) + "\n"
    def add_boid_at_pos(self,pos_vector):
        '''
        :description: adds a boid at the specified position set with a random velocity and acceleration
        :param pos_vector: a Vector containing the position of a new boid
        '''
        rv = Vector.rand  # random vector generator... has config settings see Vector class
        self.boid_list.add(Boid(pos_vector,rv(max_range=32),rv(max_range=3)))
    def delete_boid(self,id=False,index=False,random=False):
        '''
        :description: deletes a boid based on the supplied criteria by id,index,random (may pass one kew word arg at a time)
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






