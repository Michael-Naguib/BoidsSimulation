# Boids Simulation
 Boids are a way of modeling the complex flocking behavior of birds as well as many marine life 
 including schools of fish; Each Boid is governed by simple rules yet these are sufficient to generate 
 complex behaviors. These behaviors have been described by [Craig Reynolds](https://www.red3d.com/cwr/boids/)  
 who is the creator of the concept of a *boid* The rules for a simple boid simulation are: 
 - Cohesion (tendency to stick together within a threshold distance radius)
 - Separation (tendency to separate  within a threshold distance radius)
 - Alignment (tendency to follow in same direction within a threshold distance radius)   
 
 ... please refer to the [Craig's](https://www.red3d.com/cwr/boids/) website for more details

## About the Code
- When I first began coding this simulation I believed it would run fast *enough* however I soon realized
    that the simulation suffered from algorithmic complexity: Calculating the distance between every Boid was & is still 
    the slowest part of the code. It is an **O(n^2)** operation where n is the number of boids present in the simulation. 
    One of the major goals of my code was to generalize the code so that a boid could be efficiently computed in more than 
    just 2D and 3D space but also in 4D space. Future aspirations even included implementing a genetic algorithm to see 
    how to make a robust swarm of agents capable of collecting *food* from a N-Dimensional Environment. 

## Initial Design Choices 
Here were my (initial computationally-slow) design choices(note some were not so much choices as much as it was initial naivety):
+ Distance Calculations were based on a variant of Euclidean Distance I designed to determine the 
correct distance in a wrapped world geometry: using the modulo and branching with if statements was 
a constant factor slower than simple Euclidean Distance
+ As mentioned before computing the distances between every boid was **O(n^2)** which resulted in 
    latency during the live viewing of the simulation after n=80 (~150ms per distance calculation)
+ Wrote my own Vector Library (It worked well and was even fast however it creates many objects for python to manage 
    under the hood ... )
+ The simulation would be an *exact* and not an *approximate* simulation ... 
+ Grapics were drawn sequentially: this was a significantly weighted **O(c*n)**
    where c was some rather large constant ... Drawing to the screen took up a fair amount of time (~25ms for n=80)

## Revised Design Choices
After the first failed attempt I began to recode the simulation realizing the slowdowns ... Many of these improvements have been implemented:
+ (Slightly better Complexity) Calculating the Distances between all of the boids can be reduced from **O(n^2)** to 
    **O([n^2]/2 + n/2)**. This is due to the fact that the distance from a boid A to boid B is the same as B to A so 
    may redundant calculations can be eliminated using efficient iteration
+ (Dictionary Lookup) In addition to this a dictionary lookup was used instead of python's list based ... This got 
    slight (~1ms) advantages when N was large however not enough to be very better (nor worse) in the short run
+ (Squared Distance) Euclidean Distance involves taking the square root of a number however the sqrt operation 
    is relatively costly and is unnecessary in some circumstances where if you are simply comparing to a distance it 
    is much more computationally efficient to do an extra square operation vs a sqrt op; ex. *d = sqrt(x^2 + y^2)*
    comparing *d* to a distance *a* given that both *a*, *d* are distances (thus positive) the equality and such similar 
    ones ... *a<=d=sqrt(x^2 + y^2)* can be rewritten *a^2 <= d^2 = x^2 + y^2*. So to make the overall boid simulation faster only 
    the squared distance was computed when possible .The computational benefit is that we dont need the actual distance ... 
    and preforming a square operation is faster than sqrt.
+ (New Bounding Mechanism) I will define bounding as keeping the boids and their positions velocities ... etc with a 
    computationally viable viewing space. Previously I had designed the simulation to use a variant of Euclidean Distance I 
    tailored to work in a modular space however branching here and the modulo operation are costly. (Modular Space: I had 
    wanted the world geometry to wrap ... the 2D analogy being the surface of sphere appears continuous no matter where you 
    go... 3D simply looping position etc... ). This had made bounding the simulation within Double Precision numbers easy as 
    all I had to was modulo by the world bounds and this had the added benefit that the world was continuous and this 
    simplified mechanics in general, However this has a relatively high constant for computational complexity. 
    Instead I found a new way online for how to Bound the simulation. The new method was to create an unrestricted force 
    that grows in proportion to a boids distance from the word bounds and which also directs them inwards. 
    This force should be able to exceed maximum force limit placed on other rules. This bounding mechanism does not 
    guarantee that boids may not have positions outside the bounds but rather that they will always return to the bounds 
    relatively quickly ... *'a force field'*
+ (GPU Rendering) As mentioned before drawing each boid sequentially led to a significant slowdown ... python's tkinter 
    library was just not fast enough. This was rectified by using an efficient point cloud visualization library called 
    Open3D which takes advantage of the GPU to display point clouds very quickly
    in live time!
+ (Pre-compute) Although this is not so much as a speedup to the original algorithm ... this does enable the simulation 
    to be easily viewed: Instead of computing the simulation between each rendering of the frame, compute all the position 
    data ahead of time saving it to a file then play back the simulation (this worked fantastic for viewing even allowing 
    for viewing of a Slow Modulo Bounded simulation instead of a Faster Force Field Bounded Simulation! however computing 
    the simulations was still asymptotically slow... that did not change... )
+ (Neighbor Approximation) So a traditional Boid Simulation algorithm says to check to see for every rule what boids 
    count as that boid's neighbors ... i.e a certain rule could have a distance radius that includes every boid, however 
    most often this is not the case... in fact in sparse low density simulations boids may not ever have a significantly 
    large number of neighbors yet all are being checked. The simulation was sped up by approximating how many neighbors 
    were taken into account. Say for example a boid had 10 neighbors for which the forces were calculated ... the remaining 
    89 boids in 100 boid population would not be even considered despite the fact that some may be a neighbor. 
    This artificially limited the complexity of neighbor force calculations and led to a large speedup (~ 30-40ms) 
    however could cause the simulation to fail to achieve the desired behavior ... 
## Planned Design Choices:
+ (Parallel Distance Calculation) Implement a distributed version of the Distance Calculation algorithm for the boids:  
    Utilize the GPU to take advantage of the core count ... this could make significantly larger computations of N boids 
    feasible for live playback and even larger N values for precomputed. (it might be worth it also to make this a PC Core
     distributed algorithm or multi-pc distribute capable ... GPU is definitely the best start and probably all that's 
     necessary for the scope of this project)
+ (Random Neighbor Approximation) Notably unless the neighbors are tested at random before the threshold is reached
     there can be some code side effects ... potential neighbors must be selected at random without replacement till the
      threshold is met, otherwise boids distant from eachother in the storage list have significantly low chances of
       interacting through force calculations ... Randomness ensures equal chance. 
+ (Adapt the Barnes Hutt Algorithm) to make the force calculations a computationally viable approximation.
    The slowest part of the algorithm that is preventing it from working is the  **O(n^2)** distance calculation ... 
    Barnes Hutt can compute forces not cut off at certain distances in **O(n*log(n))**
+ Implement a KD-Tree to efficiently construct the distances between boids and query the neighbors ...
    ( **O(nlog(n))** to build the tree, and **log(n)** to query the neighbors however note that becomes 
    **O(nlog(n))** because we will have to query for *n* boids ... and that will have to be redone for each rule...) 
    (Despite the high constants on this type of calculation I believe it is the best approach second only to GPU 
    implementation: this is better asymptotically... GPU may just be faster for the scale of simulations we wish to do)

## Precomputed Screenshots (Modular Space Configuration)
This was calculated via the notebook ```BoidSimulation1.5.ipynb```. I had an afterthought that I could
pre-compute the simulations saving the data then view them in playback to avoid a high latency in the progression of frames. 
The images below are several screenshots taken while viewing a 3D simulation of boids. (Open3D is wonderful for panning zooming around a live simulation!!)
[Boid Sim 1]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d1.png "Boid Sim 1")
[Boid Sim 2]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d2.png "Boid Sim 2")
[Boid Sim 3]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d3.png "Boid Sim 3")
[Boid Sim 4]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d4.png "Boid Sim 4")
[Boid Sim 5]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d5.png "Boid Sim 5")

## Features to add: (progress in ```BoidSimulation2.0.ipynb```)
- Save Simulations
- Easily configure settings
- Enable Modular Space vs Force Bounding
- Pause Simulation Playback
- Color simulation based on acceleration etc...
- Adjust boid settigns in live time for live time simulation
- Human configurable and controllable boid/s/microswarm? (via arrow keys)
- GUI for saving and loading simulations /settings /etc

## TL;DR;
- Code needs work ... Begin first with GPU acceleration

