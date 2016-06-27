from sensors import sensor
import numpy as np
from math import sqrt
import cv2
import A_Pathfinding as alg

class MyAlgorithm():

    first = True

    def __init__(self, sensor, grid):
        self.sensor = sensor
        self.grid = grid
        sensor.getPathSig.connect(self.generatePath)

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) mathod for setting the path. """
    def generatePath(self):
        print "LOOKING FOR SHORTER PATH"
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        dest = (dest[1],dest[0])
        gridPos = (gridPos[1], gridPos[0])

        if self.first:
            print mapIm

            for y in range(len(mapIm)):
                for x in range(len(mapIm[0])):
                    if mapIm[y][x] > 0:
                        mapIm[y][x] = 0
                    else:
                        mapIm[y][x] = 1


            print mapIm
            self.first = False


        #TODO
        nmap = np.array(mapIm)
        pathinstructions = alg.astar(nmap, dest, gridPos)

        # Fill distance grid
        pos = len(pathinstructions)
        for inst in pathinstructions:
            self.grid.setVal(inst[1], inst[0], pos)
            pos = pos-1

        # Fill path
        for inst in pathinstructions:
            self.grid.setPathVal(inst[1], inst[0], 1)

        self.grid.setPathFinded()

        #Represent the Gradient Field in a window using cv2.imshow
        self.grid.showGrid()

    """ Write in this mehtod the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
        # Add your code here
        print "GOING TO DESTINATION"