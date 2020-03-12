#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from robitcode.timmy import robit

from vecart.coordinate import Coordinate
from vecart.vector import Vector
import math
# Write your program here

brick.sound.beep()

motor1 = Motor(Port.A)
motor2 = Motor(Port.D)
timmy = robit(motor1, motor2, Coordinate(0,0), math.pi/2)
timmy.calibrate()


wait(1000)

brick.display.clear()
timmy.move(Coordinate(0, 50))
wait(2000)

brick.display.clear()
timmy.move(Coordinate(0, 45))
wait(2000)

brick.display.clear()
timmy.move(Coordinate(0, 10))