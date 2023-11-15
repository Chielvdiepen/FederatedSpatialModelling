import random

random.seed(7)

office_width = 5
office_length = 5
office_height = 3

num_nodes = 0

# RefNodeCoords = [(0,0,0),(3,0,0),(0,3,0),(3,3,0),(6,0,0),(6,3,0),(1.5,1.5,2),(4.5,1.5,2)] # double pyramid
# RefNodeCoords = [(0,0,0),(0,4,0),(4,0,0),(4,4,0),(0,0,4),(0,4,4),(4,0,4),(4,4,4)] # cube
RefNodeCoords = [(0,0,0),(4,0,0),(3.5,2,0),(1.5,3,2),(3,3,3)] # Tetraeder + extra node
# RefNodeCoords = [(0,0,0),(4,0,0),(2.0,4,0),(2,0,4),(2,2,2)] # Tetraeder + extra node, symmetric
# RefNodeCoords = [(0,0,0),(4,0,0),(3.5,2,0),(1.5,3,2)] # Tetraeder
# RefNodeCoords = [(5,5,0),(1,2,0),(1,9.5,0),(7,9,0),(9.5,1.5,0),(15,15,0)] # Flat Umbrella + extra node
# RefNodeCoords = [(5,5,0),(1,2,0),(1,9.5,0),(7,9,0),(9.5,1.5,0)] # Flat Umbrella
# RefNodeCoords = [(7,3,0),(0,0,0),(4,0,0)] # Simple Triangle

DIM2D = False

if DIM2D:
    TAGCOORD = (1,1,0)
else:
    TAGCOORD = (1,1,1)

MAXTRIANGLES = 25
RSSI = False
PLOT = False