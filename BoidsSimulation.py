'''
# Boids Simulation
- Code By Michael Sherif Naguib
- license: MIT
- Date: 6/18/19
- @University of Tulsa
## Description:
- This file is the 'main' script which runs the entire simulation and where the configuration of the code happens.


'''
#imports
from tkinter import *
from BoidSwarm import BoidSwarm
import time
#main program
if __name__ == "__main__":
    #=== Settings
    HEIGHT = 300
    WIDTH = round(HEIGHT * 1.6180339) # preserve a nice aspect ratio by using phi= THE GOLDEN RATIO (one of my favorite numbers)
    QUANTITY=300

    #=== Setup the Graphics
    root = Tk()
    #root.overrideredirect(True)
    root.geometry('%dx%d+%d+%d' % (WIDTH, HEIGHT, (root.winfo_screenwidth() - WIDTH) / 2, (root.winfo_screenheight() - HEIGHT) / 2))
    root.bind_all('<Escape>', lambda event: event.widget.quit())
    graph = Canvas(root, width=WIDTH, height=HEIGHT, background='white')
    graph.pack()

    #=== Setup the swarm
    mySwarm = BoidSwarm(graph,WIDTH,HEIGHT)
    mySwarm.setup(quantity=QUANTITY)#Is a custom over-ridable method for configuring different types of swarms
    mySwarm.init_kinematics()#initilizes the velocities and positions

    #=== Enter the main loop
    while True:
        graph.delete(ALL)#clear the screen
        mySwarm.update_boid_positions()#runs the main calculation for the boid positions
        mySwarm.draw_swarm()#is using the reference to the graph it was passed durring init
        graph.update()#update the screen
        #time.sleep(1000)
        #print(mySwarm.boid_list[0])





'''
    example: Update Rule settings for agent agent interactions:
    
    partition the rows  indicies by which boids have the same rule so that the rule only has to be looked up once for a 
    certain range...
    
    example update rule setting: 
    
                         hive a: 1 2 3 4 5  hive b: 6 7 8 9  hive c: 10 11
                hive a:
                        1
                        2
                        3   
                        4
                        5
                hive b:
                        6
                        7
                        8
                        9
                hive c: 
                        10
                        11
                        
                        
'''
pass
#IDEAS
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