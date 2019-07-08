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
from Vector import Vector
from timeit import timeit

#main program
if __name__ == "__main__":
    #=== Settings
    HEIGHT = 600
    WIDTH = round(HEIGHT * 1.6180339) # preserve a nice aspect ratio by using phi= THE GOLDEN RATIO (one of my favorite numbers)
    DEPTH = HEIGHT
    QUANTITY=80

    #=== Setup the Graphics
    screen_dim = Vector([WIDTH,HEIGHT,DEPTH])
    root = Tk()
    root.geometry('%dx%d+%d+%d' % (WIDTH, HEIGHT, (root.winfo_screenwidth() - WIDTH) / 2, (root.winfo_screenheight() - HEIGHT) / 2))
    root.bind_all('<Escape>', lambda event: event.widget.quit())
    graph = Canvas(root, width=WIDTH, height=HEIGHT, background='white')
    graph.pack()

    #=== Setup the swarm
    mySwarm = BoidSwarm(QUANTITY,screen_dim)
    mySwarm.setup()
    @timeit
    def work():
        mySwarm.update_boid_positions()  # Run the Boid Calculation
    #=== Enter the main loop
    while True:
        graph.delete(ALL)#                  Clear the screen
        work()
        mySwarm.draw_swarm(graph)#          Draw the swarm
        graph.update()#                     Update the screen