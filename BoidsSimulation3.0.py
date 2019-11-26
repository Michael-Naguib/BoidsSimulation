'''
# Boid
- Code By Michael Sherif Naguib
- license: MIT
- Date: 11/25/19
- @University of Tulsa
- Description: Refactoring the simulation code
'''
# LOGGING CONSTANT
LOG=True

# Imports
print("Importing Deps") if LOG else None
from scipy.spatial import KDTree as KDTree  # A library for constructing and querying a KDTree
from numba import cuda, jit                 # An optimization library that can speed up code and even run on GPU
from tkinter import *                       # Python's library for creating GUI
import math                                 # Python's library for math functions etc,,,
import random                               # Python's library for random numbers
import tqdm                                 # A progress logger bar library for iteration etc...
import pptk                                 # Static 3d point cloud visualization lib for large 10mil+ datapoints
import time                                 # Python Library for time related things
import open3d                               # A library for LIDAR point cloud visualization and so much more 1k points
import numpy as np                          # a library for scientific computing
print("Finished Importing Deps") if LOG else None

