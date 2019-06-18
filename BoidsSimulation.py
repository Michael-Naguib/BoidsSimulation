'''
# Boids Simulation
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
