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
- One of the major goals of my code was to generalize the code so that a boid could be efficiently computed in more than 
    just 2D and 3D space but also in 4D space. Future aspirations even included implementing a genetic algorithm to see 
    how to make a robust swarm of agents capable of collecting *food* from a N-Dimensional Environment. 

## Initial Design Choices 
Here were my (initial computationally-slow) design choices (note some were not so much choices as much as it was initial naivety):
+ Distance Calculations were based on a variant of Euclidean Distance I designed to determine the 
    correct distance in a wrapped world geometry. Boids could from a distance calculation perspective see around 
    what appears to be a corner to us in the visualization. I found however that
    using the modulo and branching with if statements was a ** signifigant** constant factor slower than simple Euclidean Distance
+ As mentioned before computing the distances between every boid was **O(n^2)** which resulted in 
    latency during the live viewing of the simulation after n=80 (~150ms per distance calculation)
+ Wrote my own Vector Library (It worked well and was even fast however it creates many objects for python to manage 
    under the hood ... )
+ The simulation would be an *exact* and not an *approximate* simulation ... 
+ Graphics were drawn sequentially: this was a significantly weighted **O(c*n)**
    where c was some rather large constant ... Drawing to the screen took up a fair amount of time (~25ms for n=80)

## Revised Design Choices
For this revision of the code the following choices were made
+ Distance Calculation as an approximation using a KD-Tree: Unlike a exact distance calculation which takes **O(n^2)**
  a KD-Tree can be constructed in **O(n*log n )**. Querying the neighbors for an entry in the tree is **O(log n )**, thus
  for querying all of the neighbors of all of the boids the complexity is **O(n*log n )**. This benefit in complexity comes
  with a trade off however: when querying K neighbors ... the received K neighbors are approximately the nearest neighbors.
  The goal of this project was to model the complex flocking  behavior of starlings described by Craig Renyolds. 
  I learned that starlings often only consider on average six neighbors when determining their direction. Combined with the
  KD-Tree this meant that I only had to query the six nearest neighbors for each boid. This greatly improved performance.
+ Instead of allowing the boids to look around the corner with the distance function, a bounding force was created that 
  grows in proportion to the squared distance deviated from the boundaries by a boid. This force redirects the boid back 
  into a computationally manageable and observable space.
+ (Point Cloud Visualization) As mentioned before drawing each boid sequentially led to a significant slowdown ... python's tkinter 
    library was just not fast enough. This was rectified by using an efficient point cloud visualization library called 
    Open3D which takes advantage of the GPU to display point clouds very quickly in live time!
+ (Pre-compute) Although this is not so much as a speedup to the original algorithm ... this does enable the simulation 
    to be easily viewed: Instead of computing the simulation between each rendering of the frame, compute all the position 
    data ahead of time saving it to a file then play back the simulation. 

## File ```BoidsSimulation```
- This file holds the main configuration for running/loading/saving simulations
- Descriptions of settings are provided in the file next to the declaration of the setting....

## Screenshots
[Boid Sim 1]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d1.png "Boid Sim 1")
[Boid Sim 2]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d2.png "Boid Sim 2")
[Boid Sim 3]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d3.png "Boid Sim 3")
[Boid Sim 4]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d4.png "Boid Sim 4")
[Boid Sim 5]( https://raw.githubusercontent.com/Michael-Naguib/BoidsSimulation/tree/master/screenshots/bs3d5.png "Boid Sim 5")



