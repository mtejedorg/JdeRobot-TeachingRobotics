from sensors import sensor
import numpy as np
import threading
import cv2
import math



#def YUVa255(Y, U, V):
#    Y = Y * 255
#    U = (U + 0.5) / 0.8 * 255
#    V = (V + 0.6) / 1.2 * 255
#    return Y, U, V

def angle(binImg):
    first = 0
    last = 0

    j = -1
    for i in binImg[10]:
        j = j+1
        if not first and i:
            first = j

    reversebinImg = binImg[::-1]

    j = 0
    for i in reversebinImg[10]:
        j = j+1
        if not last and i:
            last = j

    linecenter = last - first
    linecenter = linecenter/2

    center = len(binImg)/2

    k = linecenter-center

    angle = k*math.pi/2/center

    return angle


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=None
        self.imageLeft=None
        self.lock = threading.Lock()

    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()

        # Convert BGR to HSV
        iLhsv = cv2.cvtColor(imageLeft, cv2.COLOR_RGB2HSV)
        # lowpass = [0, -0.2, 0.25]
        # highpass = [0.38, 0, 0.48]
        # lowpass = YUVa255(*lowpass)
        # highpass = YUVa255(*highpass)

        lowpass = (0, 207, 69)
        highpass = (10, 255, 198)

        ILhsvInRange = cv2.inRange(iLhsv, lowpass, highpass)
        mm = cv2.moments(ILhsvInRange)
        cy = mm['m10']/'m00'

        IlhsvInRange3channels = np.dstack((ILhsvInRange, ILhsvInRange, ILhsvInRange))

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        self.sensor.setV(10)
        self.sensor.setW(angle(ILhsvInRange))


        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(IlyuvInRange3channels)
        self.setLeftImageFiltered(IlyuvInRange3channels)



    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage

