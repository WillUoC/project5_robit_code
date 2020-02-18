#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from timmy import robit

from coordinate import Coordinate
from vector import Vector
import math
# Write your program here

brick.sound.beep()

motor1 = Motor(Port.A)
motor2 = Motor(Port.D)
timmy = robit(motor1, motor2, Coordinate(0,0), math.pi/2)
timmy.calibrate()
wait(1000)

timmy.move(Coordinate(20, 20))
wait(2000)

timy.move(coordinate(-30, 45))
wait(10000)


