import vector
import math

class Coordinate:

    """
        A datatype designed to store x and y coordinates and convert to vector coordinates if necessary
    """


    def __init__(self, x_coordinate, y_coordinate):
        self.__x_coordinate = x_coordinate
        self.__y_coordinate = y_coordinate
    
    """
        Returns the x coordinate
    """
    def getX(self):
        return(self.__x_coordinate)
    
    """
        Returns the y coordinate
    """
    def getY(self):
        return(self.__y_coordinate)
    
    """
        Returns both the x and y coordinates in the form of a tuple:
            (x, y)
    """
    def getCoords(self):
        return((self.__x_coordinate, self.__y_coordinate))

    """
        When given original coordinates, returns the change in x and y
            (x2 - x1, y2 - y1)
    """
    def getDeltaCoords(self, original_coordinate):

        return(Coordinate(self.__x_coordinate - original_coordinate.getX(), self.__y_coordinate - original_coordinate.getY()))
    
    
    
    """
        Convert Cartesian coordinates to Vectors;
            Coordinates stored as tuple (x, y)
            Vectors stored as tuple (theta, magnitude)
    """
    def vector(self):
        if self.__x_coordinate >= 0:
            if self.__y_coordinate >= 0:
                #Quadrant I
                theta = math.atan(self.__y_coordinate/self.__x_coordinate)
            else:
                #Quadrant II
                theta = 360 + math.atan(self.__y_coordinate/self.__x_coordinate)
        else:
            #Quadrant III & IV
            theta = 180 + math.atan(self.__y_coordinate/self.__x_coordinate)
        
        magnitude = math.sqrt(self.__x_coordinate**2 + self.__y_coordinate**2)

        return(vector.Vector(theta, magnitude))
    
    def __str__(self):
        return("({0:.2f}, {1:.2f})".format(self.__x_coordinate, self.__y_coordinate))
        