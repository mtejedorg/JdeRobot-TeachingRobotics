from sensors import sensor
import numpy as np
from math import sqrt
import cv2
import A_Pathfinding as alg
import math

def abs_2_rel_coord(x, y, rx, ry, rt):
    # Convert to relatives
    dx = x - rx
    dy = y - ry

    # Rotate with current angle
    x = dx * math.cos(-rt) - dy * math.sin(-rt)
    y = dx * math.sin(-rt) + dy * math.cos(-rt)

    return x, y


def get_angle(x,y):
    if(y>=0):
        # 0 becomes -1 and Pi becomes 1
        mod = math.sqrt(x**2 + y**2)
        ang = math.acos(x/mod)
        ang = -ang/math.pi           # Normalized
        ang = ang*2+1
        # ang = -ang
    else:
        if x > 0:
            ang = 1
        else:
            ang = -1

    return ang


def vect_2_mod_angle_car(x, y):
    mod = math.sqrt(np.square(x) + np.square(y))
    ang = get_angle(x,y)

    return mod, ang

class MyAlgorithm():

    def __init__(self, sensor, grid):
        self.sensor = sensor
        self.grid = grid
        sensor.getPathSig.connect(self.generatePath)

    first = True

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

        print "Dest: " + str(dest)
        print "Position: " + str(gridPos)

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

        print pathinstructions

        # Fill distance grid
        try:
            pos = len(pathinstructions)

            for inst in pathinstructions:
                self.grid.setVal(inst[1], inst[0], pos)
                pos = pos-1

            # Fill path
            for inst in pathinstructions:
                self.grid.setPathVal(inst[1], inst[0], 1)

            self.grid.setPathFinded()

            #Represent the Gradient Field in a window using cv2.imshow
            # self.grid.showGrid()

        except TypeError:
            print "Value given is not a valid map point"

    def moveto(self, neighborx, neighbory):
        rx = self.sensor.getRobotX()
        ry = self.sensor.getRobotY()
        rt = self.sensor.getRobotTheta()
        (x,y) = self.grid.gridToWorld(neighborx, neighbory)

        print "grids " + str(neighborx) + "," + str(neighbory) + " becomes worlds " + str(x) + "," + str(y)

        (relx, rely) = abs_2_rel_coord(x, y, rx, ry, rt)

        print "worlds " + str(x) + "," + str(y) + " becomes relative " + str(relx) + "," + str(rely)

        (mod, angle) = vect_2_mod_angle_car(relx, rely)

        print "vector " + str(relx) + "," + str(rely) + " becomes mod, angle: " + str(mod) + "," + str(angle)

        self.sensor.setV(4)
        self.sensor.setW(angle)

    """ Write in this method the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
        # Add your code here
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        x = self.sensor.getRobotX()
        print x
        y = self.sensor.getRobotY()
        print y
        (x,y) = self.grid.worldToGrid(x,y)
        print x
        print y
        myPos = self.grid.getPose()
        print myPos

        for i,j in neighbors:
            neighborx = myPos[0]+i
            neighbory = myPos[1]+j
            if self.grid.getPathVal(neighborx, neighbory)>0:
                self.moveto(neighborx, neighbory)
                break

        print "GOING TO DESTINATION"