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
    def getX():
        return(__x_coordinate)
    
    """
        Returns the y coordinate
    """
    def getY():
        return(__y_coordinate)
    
    """
        Returns both the x and y coordinates in the form of a tuple:
            (x, y)
    """
    def getCoords():
        return((__x_coordinate, __y_coordinate))

    """
        When given original coordinates, returns the change in x and y
            (x2 - x1, y2 - y1)
    """
    def getDeltaCoords(x1, y1):
        return((__x_coordinate - x1, __y_coordinate - y1))
    
    
    
    """
        Convert Cartesian coordinates to Vectors;
            Coordinates stored as tuple (x, y)
            Vectors stored as tuple (theta, magnitude)
    """
    def vector():
        if __x_coordinate >= 0:
            if __y_coordinate >= 0:
                #Quadrant I
                theta = math.atan(__y_coordinate/__x_coordinate)
            else:
                #Quadrant II
                theta = 360 + math.atan(__y_coordinate/__x_coordinate)
        else:
            #Quadrant III & IV
            theta = 180 + math.atan(__y_coordinate/__x_coordinate)
        
        magnitude = math.sqrt(__x_coordinate**2 + __y_coordinate**2)

        return(vector(theta, magnitude))
        