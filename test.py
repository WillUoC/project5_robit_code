from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from vecart.coordinate import Coordinate
from vecart.vector import Vector
from robitcode.timmy import robit

import math

import unittest

class TestRobit(unittest.TestCase):
    def test_robit_calibration(self):

    
if __name__ == '__main__':
    unittest.main()
motor1 = Motor(Port.A)
motor2 = Motor(Port.D)
timmy = robit(motor1, motor2, Coordinate(0,0), math.pi/2)