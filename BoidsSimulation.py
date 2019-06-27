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
    HEIGHT = 300
    WIDTH = round(HEIGHT * 1.6180339) # preserve a nice aspect ratio by using phi= THE GOLDEN RATIO (one of my favorite numbers)
    QUANTITY=50

    #=== Setup the Graphics
    root = Tk()
    #root.overrideredirect(True)
    root.geometry('%dx%d+%d+%d' % (WIDTH, HEIGHT, (root.winfo_screenwidth() - WIDTH) / 2, (root.winfo_screenheight() - HEIGHT) / 2))
    root.bind_all('<Escape>', lambda event: event.widget.quit())
    graph = Canvas(root, width=WIDTH, height=HEIGHT, background='white')
    graph.pack()

    #=== Setup the swarm
    mySwarm = BoidSwarm(QUANTITY,Vector([WIDTH,HEIGHT]))

    #=== Enter the main loop
    while True:
        graph.delete(ALL)#clear the screen
        mySwarm.update_boid_positions()#runs the main calculation for the boid positions
        mySwarm.draw_swarm(graph)#is using the reference to the graph it was passed durring init
        graph.update()#update the screen