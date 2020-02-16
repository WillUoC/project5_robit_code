class Vector:
    """
        A datatype to store vector coordinates
        (theta, magnitude)
        theta is in radians
    """

    def __init__(self, theta, magnitude):
        self.__theta = theta
        self.__magnitude = magnitude
    
    """
        Returns the angle of the vector in radians
    """
    
    def getTheta():
        return(__theta)
    
    """
        Returns the magnitude of the vector
    """
    def getMagnitude():
        return(__magnitude)

    """
        Returns the whole vector in tuple form:
            (theta, magnitude)
    """
    def getVector():
        return((__theta, __magnitude))
    
    """
        Convert Vectors to Cartesian Coordinates;
            Coordinates stored as tuple (x, y)
            Vectors stored as tuple (theta, magnitude)
    """
    def getCart(vector)
        xCoord = __magnitude * math.cos(__theta)
        yCoord = __magnitude * math.sin(__theta)
        return(coordinate(xCoord, yCoord))