from sensors import sensor
import numpy as np
import threading
import jderobot
import math
from Target import Target
from sensors import sensor
import numpy as np
import threading
import jderobot
import math
from Target import Target
from Parser import Parser

strength = 4


def abs_2_rel_coord(x, y, rx, ry, rt):
    # Convert to relatives
    dx = x - rx
    dy = y - ry

    # Rotate with current angle
    x = dx * math.cos(-rt) - dy * math.sin(-rt)
    y = dx * math.sin(-rt) + dy * math.cos(-rt)

    return x, y


def vect2_add (vect1, vect2):
    vect1 = list(vect1)
    vect2 = list(vect2)
    vectx = vect1[0] + vect2[0]
    vecty = vect1[1] + vect2[1]

    return vectx, vecty


def get_angle(x,y):
    if(y<=0):
        # 0 becomes -1 and Pi becomes 1
        mod = math.sqrt(x**2 + y**2)
        ang = math.acos(x/mod)
        ang = -ang/math.pi           # Normalized
        ang = ang*2+1
        ang = -ang
    else:
        if x > 0:
            ang = 1
        else:
            ang = -1

    return ang


def get_vect_mod(x,y):
    mod = math.sqrt(np.square(x) + np.square(y))
    return mod


def vect_2_mod_angle_car(x, y):
    mod = math.sqrt(np.square(x) + np.square(y))
    ang = get_angle(x,y)

    return mod, ang


# Manages the data of the laser
def parsed_laser_to_GUI(laser):
    laser_vectorized = []
    for d, a in laser:
        # (4.2.1) laser into GUI reference system
        x = d * math.cos(a) * -1
        y = d * math.sin(a) * -1
        v = [x, y]
        laser_vectorized += [v]
    return laser_vectorized


def parse_obstacles_data (laser_data):
    laser = []
    for i in range(laser_data.numLaser):
        dist = laser_data.distanceData[i] / 1000.0
        angle = math.radians(i)
        laser += [(strength/dist, angle+math.pi)]
    return laser


def get_main_obstacle(laser_data):
    closestindex = 0
    mindist = 10
    for i in range(laser_data.numLaser):
        dist = laser_data.distanceData[i] / 1000.0
        if dist < mindist:
            mindist = dist
            closestindex = i

    if mindist < 1.2:
        angle = math.radians(closestindex)
        obstacle = [(strength/mindist, angle+math.pi)]
    else:
        obstacle = [(0, math.pi/2)]

    obstacle_vect = parsed_laser_to_GUI(obstacle)

    obstacle_vect = obstacle_vect[0]

    return obstacle_vect


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=None
        self.imageLeft=None
        self.lock = threading.Lock()

        # Car direction
        self.carx = 0.0
        self.cary = 0.0

        # Obstacles direction
        self.obsx = 0.0
        self.obsy = 0.0

        # Average direction
        self.avgx = 0.0
        self.avgy = 0.0

        # Current target
        self.targetx = 0.0
        self.targety = 0.0

        # Init targets
        parser = Parser('targets.json')
        self.targets = parser.getTargets()

    def get_vect_target(self, x, y, weight):
        rx = self.sensor.getRobotX()
        ry = self.sensor.getRobotY()
        rt = self.sensor.getRobotTheta()
        target_rel = abs_2_rel_coord(x, y, rx, ry, rt)

        vectx = target_rel[0]
        vecty = target_rel[1]

        dist = get_vect_mod(vectx, vecty)

        if dist < 1:
            weight = weight * 1/dist

        vect = (vectx, vecty)

        # Applies the weight
        vectwx = vectx * weight
        vectwy = vecty * weight

        vectw = (vectwx, vectwy)

        return vect, vectw, dist

    def getNextTarget(self):
        for target in self.targets:
            if target.isReached() == False:
                return target

        return None

    def execute(self):
        self.currentTarget = self.getNextTarget()
        self.targetx = self.currentTarget.getPose().x
        self.targety = self.currentTarget.getPose().y

        # TODO
        # Calcular fuerzas
        # Fuerza yo-destino

        weight = 0.7
        target, targetw, dist = self.get_vect_target(self.targetx, self.targety, weight)

        if dist < 1:
            self.currentTarget.setReached(True)
        else:
            # Fuerza obstaculos-yo

            laser_data = self.sensor.getLaserData()
            obstacle_vect = get_main_obstacle(laser_data)

            self.obsx = obstacle_vect[0]
            self.obsy = obstacle_vect[1]

            # vector suma ==> direccion y velocidad
            # Decidir destino

            final_vect = vect2_add(targetw, obstacle_vect)

            self.carx = final_vect[0]
            self.cary = final_vect[1]

            final_vect_mod_angle = vect_2_mod_angle_car(final_vect[0], final_vect[1])

            # Traducir a angulo, velocidad

            k = 0.5

            speed = final_vect_mod_angle[0] * k
            if speed > 1.5:
                speed = 1.5

            self.sensor.setV(speed)
            self.sensor.setW(final_vect_mod_angle[1])

    # Gui functions
    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight = image
        self.lock.release()

    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        self.imageLeft = image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage = self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage = self.imageLeft
        self.lock.release()
        return tempImage

    def getCarDirection(self):
        return (self.carx, self.cary)

    def getObstaclesDirection(self):
        return (self.obsx, self.obsy)

    def getAverageDirection(self):
        return (self.avgx, self.avgy)

    def getCurrentTarget(self):
        return (self.targetx, self.targety)
