import coordinate
import math

class Vector:
    """
        A datatype to store vector coordinates
        (theta, magnitude)
        theta is in radians
    """
    __magnitude = 0
    __theta = 0

    def __init__(self, theta, magnitude):
        self.__theta = theta
        self.__magnitude = magnitude
    
    """
        Returns the angle of the vector in radians
    """
    
    def getTheta(self):
        return(int(self.__theta))
    
    """
        Returns the magnitude of the vector
    """
    def getMagnitude(self):
        return(int(self.__magnitude))

    """
        Returns the whole vector in tuple form:
            (theta, magnitude)
    """
    def getVector(self):
        return((self.__theta, self.__magnitude))
    
    """
        Convert Vectors to Cartesian Coordinates;
            Coordinates stored as tuple (x, y)
            Vectors stored as tuple (theta, magnitude)
    """
    def getCart(self, vector):
        xCoord = self.__magnitude * math.cos(self.__theta)
        yCoord = self.__magnitude * math.sin(self.__theta)
        
        return(coordinate(xCoord, yCoord))

    def __str__(self):
        return("({0:.2f}, {1:.2f})".format(self.__theta * 180/math.pi, self.__magnitude))