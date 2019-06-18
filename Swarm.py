'''
# Swarm
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/13/19
- @University of Tulsa
## Description:
- Modeling a Boids Flock (swarm) according to Separation Alignment & Cohesion and Avoidance Rules
                see: https://www.red3d.com/cwr/boids/
- well... not quite ... I had some ideas along the way:
- boid weight:  most calculations for center of mass (cohesion) assume each boid weighs the same or rather that
                each boid has the same importance... I introduce a weight factor which for a normal boid swarm
                simulation will have no effect if all weights are the same ... just a fun parameter to make a
                random distribution of weights and see what will happen.

'''

import random
import time
import arcade
import uuid
import math

#Stores all the associated information with a boid
class Boid():
    def __init__(self,x_0,y_0,color=(0,0,0)):
        '''
        :description: Creates a Boid (https://www.red3d.com/cwr/boids/)                     (side note! i just figured out pycharm can add in docstrings! first time using these... fairly useful!)
        :param x_0: is the initial x coordinate of the boid
        :param y_0: is the initial y coordinate of the boid
        :param color: defaults to black is a tuple of three integers specifying an rgb color of the boid's drawing example (255,255,255)
        '''
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
        #Descriptive information
        self.id= uuid.uuid4()#          a unique id for the boid
        self.name=None#                 an optional name for the boid
        self.type=None#                 specifies a type of boid (i.e there may be different interacting swarms)
        #Extra Information:
        self.weight=1#                  an idea i had --> some boids have more importance when calculating cohesion for center of mass see document description comment (1=identity and no effect if all are 1)
        #(planned) Statistical Information: idea=keep track of # of close calls with predators ... use genetic algorithm for update rule params

#Helps create and run a boid swarm
class BoidSwarm():
    #=== internal setup for this class do not modify
    def __init__(self,max_x,max_y):
        '''
        :description: internal class initialization for the swarm
        :param max_x: the maximum world bound in the x direction... ( boid x cordinates always on [0,max_x] )
        :param max_y: the maximum world bound in the y direction... ( boid y cordinates always on [0,max_y] )

        usage:

        bs = BoidSwarm(160,100)
        bs.setup()# you must override this
        bs.init_kinematics()# you must override this

        ... loop
        bs.update_boid_positions

        '''
        #=== Update Rules & Boid Storage
        self.boid_list=[]#      stores all the boid objects; relative order in this list should be preserved!
        self.update_rules=[]#   stores every function that will be passed the boid_list ==> note should return a list of velocity update tuple pairs [(vx,vy)...]
        #Stores the modular wrapping bounds for the spherical surface  (modular 2d space) assumes min_x,min_y = 0
        self.max_y = max_y
        self.max_x = max_x
        #Stores the Markov Matricies for the relationships between each Update Rule hyper-parameters between every boid (technically its not quite a markov matrix ... as sigma prob != 1 necessarily but it does capture relationship informaiton
        self.markov_tensor=[]# store each matrix for each function in here ... --> order corresponds to the functions prder in update_rules
        #Stores the Partitioning scheme for each Markov Matrix (i.e  indicies a thru b  have same update rule for [(rnage1,range2...)] for rule P
        #         [ scheme for function 0, scheme for function 1,...]  where a scheme ==> [ {[a,b] ,([r0,r1],[r2,r3],...])} , {[c,d] ,([r0,r1],[r2,r3],...])},... ]  ==> that is a scheme for 1 function
        #         a<b<c<d ==> they are in order ... every partition must be accounted for...
        self.markov_partitioning_tensor=[]

    #=== (override setup!! or use this default this is where you build custom types of swarms...)
    def setup(self):
        '''
        :description: Sets up the initial state of the swarm i.e how many boids configured  color ... etc... using [...] update rules and hyper parameters
        '''
        #Add the desired update rules (functions  accept boidlist as param ... returns velocity update ... use a lambda abstraction to bind hyper-parameters
        cohere = lambda boid_list: BoidSwarm.COHESION(boid_list,percent_strength=0.01,modular_space_params=(self.max_x,self.max_y))
        seperate = lambda boid_list: BoidSwarm.SEPARATION(boid_list, percent_strength=0.01,modular_space_params=(self.max_x, self.max_y))
        align = lambda boid_list: BoidSwarm.ALIGNMENT(boid_list, percent_strength=0.01,modular_space_params=(self.max_x, self.max_y))
        self.update_rules.extend([cohere,seperate,align])

        #Create the swarm by appending the new boid to self.boid_list: example generate 100 boids with random positions within a certian bound
        max_x=160#       if implementing this in a graphical sense these could be the screen bounds...
        max_y=100#
        for i in range(100):
            rand_x=random.randint(0,max_x)#assuming min_x=0
            rand_y = random.randint(0, max_y)#assuming min_y=0
            self.boid_list.append(Boid(rand_x,rand_y)) #NOTE! velocity inits to a random dydt dxdy each on [-1,1] but if that is too slow for a starting velocity it should be set before appending

    #=== (OVERRIDE) inititilize the kinematics for the boid: starting position and starting velocity
    def init_kinematics(self):
        '''
        :description: this function should iterate over every boid in the self.boid list and set an initial position and
                      velocity
        '''
        #Random strategy:
        for current_boid_index in range(0,len(self.boid_list)):
            #update
            ref=self.boid_list[current_boid_index]
            #init to a random velocity and a random position
            ref.dydt = random.randint(-1,1)
            ref.dxdt = random.randint(-1,1)
            ref.x = random.randint(self.max_x)
            ref.y = random.randint(self.max_y)

    #=== (RUN) essentially advances the next time step
    def update_boid_positions(self):
        '''
        :description: updates all of the velocities and positions of the boids in the boid list according to the
                      list of present update rules stored in update_rules
        '''

        #Make no mistake this one line is a massive calculation....
        velocity_updates = [rule(self.boid_list) for rule in self.update_rules]# [  {v1={x,y},v2},{etc...}  ]

        for current_boid_index in range(0,len(self.boid_list)):
            #intit
            velocity_x = 0
            velocity_y = 0
            #sum for each rule
            for i in range(0,len(velocity_updates)):
                velocity_x += velocity_updates[i][current_boid_index][0]
                velocity_y += velocity_updates[i][current_boid_index][1]
            #update
            ref=self.boid_list[current_boid_index]
            #change position and velocity...
            ref.dydt = velocity_x
            ref.dxdt = velocity_y
            ref.x = ref.x + velocity_x
            ref.y = ref.y + velocity_y

    #=== Resolve Hyper Parameters:
    def _resolve_hyper_parameters(self,range0,range1):
        pass

    #=== Predefined Update rules! these may be used withine the BoidSwarm ... custom rules can be added...
    @staticmethod
    def COHESION(boid_list,percent_strength=0.01,modular_space_params=False):
        '''
        :description: Calculates the updates to the velocities of each boid in the swarm
                      each boid only considers other boids in its own influence_radius
                      each boid is excluded when calculating its own velocity update (e.g its own center of mass is not taken into account)
        :param boid_list: a reference to the list of boids
        :param percent_strength: defaults to 0.01; determines how strong the cohesion takes effect as a percentage 0.01 = 1%
        :param modular_space_params: defines the (max_x,max_y) for the space assuming it loops (is used in the calcuations to determine euclidian distance)
        :return: a list of velocity updates for each boid ... ex. [(vx_0,vy_0),(vx_1,vy_1),...,(vx_n,vy_n)] (order should match the boid it references)
        '''
        velocity_update_list=[]#store the updated velocities

        '''
        Calculate the center of mass for the boid 
            (this would be a great place to accelerate using multiple cores or distribute computation)
        Algorithm: (for each boid)   O(n^2) ouch.... but can be easily run in parallel ^^ 
            (1) compute edge distance between itself and all other boids keeping track of each index associated with edge [edge dist,boid index]
            (2) filter by edge excluding all items where  edge distance > influence_radius and index != itself (the current boid we are computing for)
            (3) calculate velocity update...  v = *percent/n_items)*sigma (weight)*position_vec
        '''
        #define a function for edge distance between two boids (euclidian distance)
        edge_distance = lambda boid0,boid1: BoidSwarm.euclidian_distance(boid0,boid1,modular_space=modular_space_params)

        #Iterate over all the boids in the list to build the velocity out
        for current_boid_index in range(0,len(boid_list)):
            #compute the edge distance and filter keeping a running count and weighted sum
            w_sum_x=0
            w_sum_y=0
            count=0
            for i in range(0,len(boid_list)):
                #Note for future use: if cohesion is a per swarm property that is to say that cohesion exists differently (relative varying ammounts)
                #or not at all between swarms in the latter case the algorithm could be sped up by partitioning the swarm space so it does not have t
                #to iterate over known partitions for which it does not interact....
                if i!=current_boid_index:#ensure we dont include the current boid...
                    #compare the current to every other in the list
                    edge_dist = edge_distance(boid_list[current_boid_index],boid_list[i])
                    #compare the edge distance to influence radius of the current boid
                    if edge_dist <= boid_list[current_boid_index].influence_radius:
                        count+=1
                        w_sum_x+=boid_list[i].x*boid_list[i].weight
                        w_sum_y+=boid_list[i].y*boid_list[i].weight
            #Now multiply weighted sum by percent strength/(number of items =count)    if number of items=0 then assign 0 for the update = no effect on velocity
            v_update_x =  (percent_strength/count)*(w_sum_x) if count !=0 else 0
            v_update_y =  (percent_strength/count)*(w_sum_y) if count !=0 else 0
            #append the result
            velocity_update_list.append((v_update_x,v_update_y))

        #return the velocity updates
        return velocity_update_list
    @staticmethod
    def SEPARATION(boid_list,percent_influence_range=0.5,modular_space_params=False):
        '''
        :description: determines the velocity update for separation of boids
        :param boid_list: a reference to the list of boids
        :param percent_influence_range: determines how far another boid must be to consider for this function  ex influence range =5 ==> if percent=0.4=40% ==> then distance <2 is considered
        :param modular_space_params: defines the (max_x,max_y) for the space assuming it loops (is used in the calcuations to determine euclidian distance)
        :return: a list of velocity updates for each boid ... ex. [(vx_0,vy_0),(vx_1,vy_1),...,(vx_n,vy_n)] (order should match the boid it references)
        '''
        velocity_update_list = []  # store the updated velocities
        # define a function for edge distance between two boids (euclidian distance)
        edge_distance = lambda boid0, boid1: BoidSwarm.euclidian_distance(boid0,boid1,modular_space=modular_space_params)
        # Iterate over all the boids in the list to build the velocity out
        for current_boid_index in range(0, len(boid_list)):
            min_distance = boid_list[current_boid_index].influence_radius*percent_influence_range
            v_update_x = 0
            v_update_y = 0
            for i in range(0,len(boid_list)):
                if (current_boid_index !=i) and edge_distance(boid_list[current_boid_index],boid_list[i])<=min_distance:#ensure it is not itself and make sure it is in the min_diatance range
                    v_update_x = v_update_x - (boid_list[current_boid_index]-boid_list[i])
                    v_update_y = v_update_y - (boid_list[current_boid_index]-boid_list[i])
            velocity_update_list.append((v_update_x,v_update_y))
        return velocity_update_list
    @staticmethod
    def ALIGNMENT(boid_list,percent_strength=1/8,modular_space_params=False):
        '''
        :description: calculates the alignment velocity updates!
        :param percent_strength: the strength of how much a boid is influenced by the aligmnet or perceived velocities of the other boids
        :return:a list of velocity updates for each boid ... ex. [(vx_0,vy_0),(vx_1,vy_1),...,(vx_n,vy_n)] (order should match the boid it references)
        '''
        velocity_update_list = []  # store the updated velocities
        # define a function for edge distance between two boids (euclidian distance)
        # Iterate over all the boids in the list to build the velocity out
        for current_boid_index in range(0, len(boid_list)):
            v_update_x = 0
            v_update_y = 0
            count=0
            for i in range(0, len(boid_list)) :
                proximity=BoidSwarm.euclidian_distance(boid_list[i],boid_list[current_boid_index], modular_space=modular_space_params)
                if (current_boid_index != i) and proximity<boid_list[current_boid_index].influence_radius:
                    v_update_x = v_update_x +boid_list[i].dxdt#x velocity
                    v_update_y = v_update_y +boid_list[i].dydt#y velocity
                    count+=1
            v_update_x= (v_update_x-boid_list[current_boid_index].x)*percent_strength/count if count != 0 else 0
            v_update_y= (v_update_y-boid_list[current_boid_index].y)*percent_strength/count if count != 0 else 0
            velocity_update_list.append((v_update_x, v_update_y))
        return velocity_update_list

    #=== Useful methods
    @staticmethod
    def euclidian_distance(boid0,boid1,modular_space=False):
        '''

        :param boid0: a boid with an x and a y cordinate   ==> boid.x boid.y
        :param boid1: a boid with an x and a y cordinate
        :param modular_space:  defaults to false ==> computes euclidian distance assuming the topography 'wraps' ...
                               pass (x_max,y_max) where it is assumed the world is bounded by min_x=0,min_y=0 and where
                               y_max and x_max are the maximum allowed x and y_positions within the world
        :return: distance as float
        '''
        if not modular_space:
            #Normal implemetation of euclidian distance
            return math.sqrt(math.pow(boid0.x-boid1.x,2)+math.pow(boid0.y-boid1.y,2))
        else:
            #use the equation i derived for modular space euclidian distance   min(|x1-x2|,max_x - |x1-x2|)   same applies to y (note the order is not specific for x2-x1 or x1-x2 but it must be consistant
            #X
            delta_x = abs(boid0.x-boid1.x)
            x_comp = min(delta_x,modular_space[0]-delta_x)#x component
            #Y
            delta_y = abs(boid0.y - boid1.y)
            y_comp = min(delta_y, modular_space[1] - delta_y)#y component
            return math.sqrt(math.pow(x_comp,2)+math.pow(y_comp,2))

    '''
    TODO: code Seperation, Alignment, Avoidence, Wind, Perching, Avoid_Zone, (Scatter)
    TODO: find a way to may the 2d euclidian space in a modular space  x=(x%max_x) 
          etc... take this into account when computing euclidian distance -->
          used in cohesion ... the influence_radius should take into account the zones maped by the modular world
    TODO: add distribution speedups and utilize multiple cores: each update rule can be computed independantly....
          and often much of what is done in the update rule can also be computed independantly
    TODO: as the specifics of boid-type to boid-type interaction have not been writen yet (i.e one fears the other and avoids while
          one seeks the other prey-predator relationship) I think using a markov matrix to represent internal seperation as well as 
          seperation values if necessary between other swarms is a good way of keeping track of the information
    TODO: a lot of the above computation for boid-type to boid-type interactions concering the update rules could be streamlined to be more efficient
          if in the beginning each swarm is partioned in the list of all the swarms ... i.e if two swarms do not interact for a certain update rule
          then that partition does not need to be even iterated over... in the case of the cohesion update rule this would improve filtering: i.e instead
          of computing for every boid in the population which is O(n^2) if it is  known that two subsets (partitions) do not interact then 
          this computation instantly becomes O(m^2) where m<=n ( m is the size of some subset of the entire population size n) the markov matrix
          could easily store these interaction levels as constants.... 
    TODO: idea!!!! what if the prisoner's dilemma was implemented --> have different swarms of predators --> each has a slightly random grab range which
          is sometimes more than their prey's detection range, and sometimes less based on a certian % and to a certain degree (customizible)  (note hunters
          also have a perception range which is much wider allowing them to track prey... add in a random variable which may cause them to loose range for a
          while --> loose tracking ... idea say if more hunters in group then this probability decreases? or increase range? an idea give advantages to lonewolf=speed?)==>
          adapt predators to cohear,seperate, align according to a genetic algorithm (gene for cohear,seperate,align... as well as a seperate gene for 
          (cooperate or defect) with a different mutation rate     ==>cohear,seperate,align are like tags! except the union space is not explicitly known!
          so by evolving behaviors ==> my guess is only if each update rule has a unique enough influence will the population become pareto optimal...
          (c.f research result sen paper this brings up an interesting question... pareto optimal solutions can only be converged upon if each of the 
          functions has a different enough effect --> if they don't ==> non pareto optimal
          )     OK now for the rewards==> setup a prisoners dilemma matrix for rewards -> generalize this to N hunters -->
          the number of hunters in a group is determiend by how many hunters are in a graph network of hunters --> i.e hunter a is in hunter b 's influence_range
          is in hunter c's influence _range while a is not necessarily in c's range    the group is consided a,b,c    ==> hunters that 'eat' a prey recieve a predefined
          reward according to they prey type ... some prey can be faster than others (hmmm this could simulate heavy prey vs light prey ==> their center of mass evolve them too!
          to survive the hunters --> group immunity of some sort......       ok when a hunter captures/eats a prey they get the corresponding reward... and the 
          prey is eliminated from the search space... for now we will assume that each time the world is init there will be #P clusterings of prey (cluster may very in type but not homogeniaty)
           and #Z clusters of predators (similarly differing in type but not homogeniaty) and furthermore that all these clusters are well spread out so as to not
           immediatly interact.... according to the rules barriers can be set within the space ... try making long corredors and seeing what happens ... just an idea
           as mentioned before the interactions between partitions should be kept track of using the markov matrix but note just as some predators are either defective or cooperative withine
           their own boid type they could have another gene which detemines this value for interaction with other predators types --> possibly at random times even instead of evolving the 
           entire subset cluster of predators predefined by boid type   --> evolve according to the frequency of certain graph interaction structures --> for example    where each hunter is a node 
           withine a subraph -->     keep track of a hunter's influence_interactions as well as how long that chain extends ... a discount factor for each non direct node --> i.e    
           hunters a-b-c and a-d-c and b-d note   c would get a discount factor or something for joining a's group??? now that i think about it captures by a hunter might not need to be determiend
           by immediate graph structure when the capture happens but more by a combination of that as well as the group structure --> what defines group structure tho... i think it might 
           have something to do with repeated interactions?      The idea here is to cross breed different boid predator types by the natural group associations that they form but also not
           simply link every type of predator out there in one graph structure... how about this: (im thinking a k means algoritm may be applicable in some way..) but lets just say for now that 
           if at the 'evolution' time if hunter a is the same boid type as hunter b then they are in a group otherwise if hunter a is not of the same type then only if hunter a is spatially close to hunter
           b then is it accepted.... remember there is a mutal fighting element between groups... .there should also be rewards for capturing/eat other types of predators ==> defined by reward
           as well as a gene indicating whether the agent cooperates or defects(eats) between other groups not just its own...
           I REST MY CASE! this will take some serious time to code but it will be worth it... computation will have to be streamlined to be run in parallel ... 
           a framework for this should be constructed... not mearly a highly specific implementation although all the bells and wistles should be added later... if not specifically
           planned for when coding this... i  think the biggest speedups will involve partitioning, parallel computation of update rules and quick hardware in general.... partitioning 
           will help fix population size; it should be noted that   as trust.io or that site where i found a graphical explanation of the prisoners dilemma--> in an email.... they mention
           different versions vs mearly cooperate/defect --> copycat,copykitten,random,etc.... these might each be a good basis for different hunter types... idk maybe make a gene which switches
           between similar behaviors of these    
    '''

#main
if __name__ =="__main__":

