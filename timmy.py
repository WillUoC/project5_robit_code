import math
from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

from vector import Vector
from coordinate import Coordinate

class robit:
    """
        Control the robot's movements and allow the robot to
    keep track of where it is at all times.
    """

    __MAX_SPEED = 500 #500 #%
    __WHEEL_RADIUS = 3.5 #cm
    __TURNING_RADIUS = 6 #cm
    __pos = []
    __steer_offset = 0

    __left_motor = None
    __right_motor = None
    __current_angle = 0

    __PROPORTIONAL_CONSTANT = 0.09
    __INTEGRAL_CONSTANT = 0.0000003
    __DERIVATIVE_CONSTANT = 650

    __TURNING_SPEED = 50


    def __init__(self, left_motor, right_motor, starting_pos, starting_angle):
        self.__left_motor = left_motor
        self.__right_motor = right_motor
        self.__pos.append(starting_pos)
        self.__current_angle = starting_angle

    
    """
        Drive the robit with the pid loop until the most accurate possible calibration is acquired
    """
    def calibrate(self):
        pidP = 0
        pidI = 0
        pidD = 0
        correction = 2 #Initial value arbitrarily decided to enable the initial loop condition

        last_error = 0
        error = 0
        self.__left_motor.reset_angle(0)
        self.__right_motor.reset_angle(0)

        self.__runMotors()
        wait(2000)
        brick.sound.beep()

        loop_time = 100
        while math.fabs(correction) > 1:
            
            #PID loop

            self.__runMotors()
            error = self.__getMotorError()

            pidP = error
            pidI += error * loop_time * self.__MAX_SPEED
            pidD = (error - last_error) / (loop_time * self.__MAX_SPEED)

            correction = pidP * self.__PROPORTIONAL_CONSTANT + pidI * self.__INTEGRAL_CONSTANT + pidD * self.__DERIVATIVE_CONSTANT
           
            self.__offset_steer(correction)
            self.__printToScrn("Correction: {}".format(correction))

            wait(loop_time)
           
            last_error = error

        brick.sound.beep()
        self.__printToScrn("Calibrated!")
        self.__stopMotors(True)
    """
        Move to specified coordinates
    """
    def move(self, coordinate):
        deltaCoords = coordinate.getDeltaCoords(self.__pos[-1])
        deltaVector = deltaCoords.vector()

        self.__printToScrn("Moving by: {0}".format(deltaCoords))
        self.__printToScrn("Vector: {0}".format(deltaVector))
        self.__turntoangle(deltaVector.getTheta())
        brick.sound.beep()
        self.__drive(deltaVector.getMagnitude())
        brick.sound.beep()
        self.__pos.append(coordinate)
    
    """
        Move to specified coordinates, backwards
        this is only to pass a subtask
    """
    def reverse(self, coordinate):
        return(None)
  
    """
        Internal function to turn to specified angle
    """
    def __turntoangle(self, angle):
        deltaTheta = angle - self.__current_angle

        self.__left_motor.reset_angle(0)
        self.__right_motor.reset_angle(0)

        deltaLeftPos = (90 * deltaTheta * self.__TURNING_RADIUS / (math.pi * self.__WHEEL_RADIUS))
        deltaRightPos = -deltaLeftPos

        self.__left_motor.run_target(self.__TURNING_SPEED, deltaLeftPos, Stop.BRAKE, False)
        self.__right_motor.run_target(self.__TURNING_SPEED, deltaRightPos, Stop.BRAKE, True)

        self.__current_angle = angle
   
   
    """
        Internal method to drive a specified distance
    """
    def __drive(self, distance):
        pidP = 0
        pidI = 0
        pidD = 0

        deltaMotorPos = (180 * distance) / (math.pi * self.__WHEEL_RADIUS)
        
        last_error = 0
        error = 0

        self.__left_motor.reset_angle(0)
        self.__right_motor.reset_angle(0)

        loop_time = 1

        while (self.__left_motor.angle() + self.__right_motor.angle()) / 2 < deltaMotorPos:
            
                
            #PID loop

            self.__runMotors()
            error = self.__getMotorError()

            pidP = error
            pidI += error * loop_time * self.__MAX_SPEED
            pidD = (error - last_error) / (loop_time * self.__MAX_SPEED)

            correction = pidP * self.__PROPORTIONAL_CONSTANT + pidI * self.__INTEGRAL_CONSTANT + pidD * self.__DERIVATIVE_CONSTANT
           
            self.__offset_steer(correction)
            self.__printToScrn("Correction: {}".format(correction))

            wait(loop_time)
           
            last_error = error

        self.__stopMotors(True)


    """
        Increase/decrease the steer
    """
    def __offset_steer(self, offset_adder):

        if self.__steer_offset + self.__MAX_SPEED + offset_adder < 0:
            self._steer_offset = -self.__MAX_SPEED
        elif self.__steer_offset + self.__MAX_SPEED + offset_adder > 100:
            self.__steer_offset = self.__MAX_SPEED
        else:
            self.__steer_offset += offset_adder
            

    """
        Run both motors with steer offset
    """
    def __runMotors(self):
        #self.__printToScrn("Steer Offset: {}".format(self.__steer_offset))
        self.__left_motor.run(self.__MAX_SPEED - self.__steer_offset)
        self.__right_motor.run(self.__MAX_SPEED + self.__steer_offset)
    
    """
        Stop running the motors
            brake if true, else coast
    """
    def __stopMotors(self, brake):
        self.__left_motor.stop(Stop.COAST) if brake == False else self.__left_motor.stop(Stop.BRAKE)
        self.__right_motor.stop(Stop.COAST) if brake == False else self.__right_motor.stop(Stop.BRAKE)

    
    """
        Get the discrepancy between the motor angles
            left veer is negative, right veer is positive
    """
    def __getMotorError(self):
        return(self.__left_motor.angle() - self.__right_motor.angle())
    
    def __printToScrn(self, text):
        print(text)
        #brick.display.text(text)
    
    def __clrScrn(self):
        brick.display.clear()