from sensors import sensor
import numpy as np
import threading
import cv2

lastW = 0
KP = 1
KD = 1

def massCenter(binImg):

    #binImg= binImg[:len(binImg)/2]

    mm = cv2.moments(binImg, True)

    center = len(binImg[1])/2

    if (mm['m00'] != 0):
        masscentery = mm['m01']/mm['m00']
        masscenterx = mm['m10']/mm['m00']
    else:
        masscentery = center;
        masscenterx = center;

    return masscenterx

def massCenter2Cams(binImgLeft, binImgRight):
    massCenterLeft = massCenter(binImgLeft)
    massCenterRight = massCenter(binImgRight)

    massCenterMean = massCenterLeft+massCenterRight
    massCenterMean = massCenterMean/2

    return massCenterMean

def angle2Cams(binImgLeft, binImgRight):
    massCenterLeft = massCenter(binImgLeft)
    massCenterRight = massCenter(binImgRight)

    centerLeft = len(binImgLeft[1])/2
    centerRight = len(binImgRight[1])/2


    leftDif = massCenterLeft-centerLeft
    rightDif = massCenterRight-centerRight

    meanDif = (leftDif+rightDif)/2

    meanCenter = (centerLeft+centerRight)/2

    currentW = - KP * meanDif/meanCenter

    lastW = currentW

    diffCorr = KD * (abs(lastW) - abs(currentW))

    if currentW != 0:
        sign = currentW/abs(currentW)
    else:
        sign = 1

    absangle = abs(currentW) - diffCorr

    angle = sign * absangle

    if abs(angle) > 1:
        angle = angle/abs(angle)

    return angle

def getSpeed(w, minSpeed = 1, maxSpeed = 10):
    if (minSpeed > maxSpeed):       # Prevents wrong args
        aux = minSpeed
        minSpeed = maxSpeed
        maxSpeed = aux

    if (minSpeed == maxSpeed):
        return minSpeed
    else:
        speedRange = maxSpeed - minSpeed
        invW = 1-w
        speed = abs(invW)*speedRange+minSpeed
        return speed

def getSpeedByCams(binImgLeft, binImgRight, minSpeed = 1, maxSpeed = 5):
    if (minSpeed > maxSpeed):
        aux = minSpeed
        minSpeed = maxSpeed
        maxSpeed = aux

    if (minSpeed == maxSpeed):
        return minSpeed
    else:
        dest = massCenter2Cams(binImgLeft[260:300], binImgRight[260:300])
        current = massCenter2Cams(binImgLeft[400:450], binImgRight[400:450])
        movement = dest-current
        if (abs(movement) < 10):
            return maxSpeed
        else:
            return minSpeed


        """
        if(abs(movement) > len(binImg[0]/4)):
            return minSpeed
        else:
            speedRange = maxSpeed - minSpeed
            invMovement = len(binImg[0]/4)-movement
            normInvMovement = invMovement/len(binImg[0]/4)
            speed = normInvMovement*speedRange+minSpeed
            return speed
        """


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

        iLhsv = cv2.cvtColor(imageLeft, cv2.COLOR_RGB2HSV)
        iRhsv = cv2.cvtColor(imageRight, cv2.COLOR_RGB2HSV)

        lowpass = (0, 207, 69)
        highpass = (10, 255, 198)

        ILhsvInRangeLeft = cv2.inRange(iLhsv, lowpass, highpass)
        ILhsvInRangeRight = cv2.inRange(iRhsv, lowpass, highpass)

        IlhsvInRange3channelsLeft = np.dstack((ILhsvInRangeLeft, ILhsvInRangeLeft, ILhsvInRangeLeft))
        IlhsvInRange3channelsRight = np.dstack((ILhsvInRangeRight, ILhsvInRangeRight, ILhsvInRangeRight))

        # Add your code here
        print "Runing"

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        w = angle2Cams(ILhsvInRangeRight, ILhsvInRangeLeft)
        v = getSpeed(w, 2, 2)
        self.sensor.setV(v)
        print 'speed: ' + str(v) + ', angle: ' + str(w)

        self.sensor.setW(w)

        #SHOW THE FILTERED IMAGE ON THE GUI
        self.setRightImageFiltered(IlhsvInRange3channelsRight)
        self.setLeftImageFiltered(IlhsvInRange3channelsLeft)



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

