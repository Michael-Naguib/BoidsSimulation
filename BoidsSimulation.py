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
from Vector import Vector

#main program
if __name__ == "__main__":
    #=== Settings
    HEIGHT = 200
    WIDTH = round(HEIGHT * 1.6180339) # preserve a nice aspect ratio by using phi= THE GOLDEN RATIO (one of my favorite numbers)
    DEPTH = HEIGHT
    QUANTITY=50

    #=== Setup the Graphics
    screen_dim = Vector([WIDTH, HEIGHT, DEPTH])
    root = Tk()
    root.geometry('%dx%d+%d+%d' % (WIDTH, HEIGHT, (root.winfo_screenwidth() - WIDTH) / 2, (root.winfo_screenheight() - HEIGHT) / 2))
    root.bind_all('<Escape>', lambda event: event.widget.quit())
    graph = Canvas(root, width=WIDTH, height=HEIGHT, background='white')
    #graph._my_z_hack = DEPTH#VERY HACKY... no advise...
    graph.pack()

    #=== Setup the swarm
    mySwarm = BoidSwarm(QUANTITY,screen_dim)
    mySwarm.setup()

    #=== Enter the main loop
    while True:
        try:
            graph.delete(ALL)#                  Clear the screen
            mySwarm.update_boid_positions()#    Run the Boid Calculation
            mySwarm.draw_swarm(graph)#          Draw the swarm
            graph.update()#                     Update the screen
        except BaseException as e:#             Handle Error & Exit
            print("The program has terminated")
            print(str(e))#log error
            break
