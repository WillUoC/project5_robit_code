class robit:
    import math
    
    from pybricks import ev3brick as brick
    from pybricks.ev3devices import Motor
    from pybricks.parameters import Port, Stop

    """
        Control the robot's movements and allow the robot to
    keep track of where it is at all times.
    """

    __MAX_SPEED = None #TODO: Decide maximum power
    __WHEEL_RADIUS = None #TODO: Input measured wheel radius
    __TURNING_RADIUS = None #TODO: Input measured turning radius
    __pos = []
    __steer_offset = 0

    __PROPORTIONAL_CONSTANT = 0
    __INTEGRAL_CONSTANT = 0
    __DERIVATIVE_CONSTANT = 0


    def __init__(self, left_motor, right_motor, starting_pos, starting_angle):
        self.__left_motor = left_motor
        self.__right_motor = right_motor
        self.__pos.append(starting_pos)
        self.__current_angle = starting_angle

    
    """
        Drive the robit with the pid loop for a certain amount of time to get a steering offset
    """
    def calibrate():
        pidP = 0
        pidI = 0
        pidD = 0
        correction = 2

        last_error = 0
        error = 0

        __left_motor.reset_angle(0)
        __right_motor.reset_angle(0)

        while correction > 1:
            #PID loop

            __runMotors()
            error = __getMotorError()

            pidP = error
            pidI += error
            pidD = error - last_error

            correction = pidP * __PROPORTIONAL_CONSTANT + pidI * __INTEGRAL_CONSTANT + pidD * __DERIVATIVE_CONSTANT
           
            __offset_steer(correction)
           
            last_error = error

        __stopMotors(True)
        
    """
        Move to specified coordinates
    """
    def move(coordinate):

    """
        Move to specified coordinates, backwards
        this is only to pass a subtask
    """
    def reverse(coordinate):

    """
        Internal function to turn to specified angle
    """
    def __turntoangle(angle):

    """
        Internal method to drive a specified distance
    """
    def __drive(distance):

    """
        Increase/decrease the steer
    """
    def __offset_steer(offset_adder):
        if __MAX_SPEED + __steer_offset > 100 or __MAX_SPEED - __steer_offset < 0:
            __steer_offset = min(100 - __MAX_SPEED, __MAX_SPEED)
        
        __steer_offset = __steer_offset + offset_adder

    """
        Run both motors with steer offset
    """
    def __runMotors():
       __left_motor.run(__MAX_SPEED - __steer_offset)
       __right_motor.run(__MAX_SPEED + __steer_offset)
    
    """
        Stop running the motors
            brake if true, else coast
    """
    def __stopMotors(brake):
        __left_motor.stop(Stop.Coast) if brake = False else __left_motor.stop(Stop.Brake)
        __right_motor.stop(Stop.Coast) if brake = False else __right_motor.stop(Stop.Brake)

    
    """
        Get the discrepancy between the motor angles
            left veer is negative, right veer is positive
    """
    def __getMotorError():
        return(__left_motor.angle() - __right_motor.angle())