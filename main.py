#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import time as t

# Creating objects
ev3 = EV3Brick()

# Initializing motors.
leftMotor = Motor(Port.C, Direction.COUNTERCLOCKWISE)
rightMotor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

# Defining class of the robot.
class RallyRobot():
    def __init__(self, blackRight, whiteRight, blackLeft, whiteLeft):
        # Storing the black and white threshold values
        self.blackRight = blackRight
        self.whiteRight = whiteRight
        self.blackLeft = blackLeft
        self.whiteLeft = whiteLeft

        # Initializing color sensors.
        self.rightSensor = ColorSensor(Port.S1)
        self.leftSensor = ColorSensor(Port.S4)

        # Initializing the drivebase.
        self.robot = DriveBase(leftMotor, rightMotor, wheel_diameter=55.0, axle_track=170.0)

        # Defining proportional gain, drive speed and turn speed
        self.PROPORTIONAL_GAIN = 9
        self.DRIVE_SPEED = 150 
        self.TURN_SPEED = 100

    def handleIntersection(self):
        # Stop the robot at intersection
        self.robot.stop()
        t.sleep(1)



    def followLine(self):
        while True:
            # Read color reflections from sensors
            right_reflection = self.rightSensor.reflection()
            left_reflection = self.leftSensor.reflection()

            # Calculate thresholds
            thresholdRight = (self.blackRight + self.whiteRight) / 2
            thresholdLeft = (self.blackLeft + self.whiteLeft) / 2

            # Calculate deviations
            deviationRight = right_reflection - thresholdRight
            deviationLeft = left_reflection - thresholdLeft

            # Determine the overall deviation for line following
            if right_reflection < thresholdRight:  # Right sensor sees black
                # Move to the left
                turn_rate = -self.PROPORTIONAL_GAIN * deviationRight
                self.robot.drive(self.TURN_SPEED, turn_rate)
            elif left_reflection < thresholdLeft:  # Left sensor sees black
                # Move to the right
                turn_rate = self.PROPORTIONAL_GAIN * deviationLeft
                self.robot.drive(self.TURN_SPEED, turn_rate)
            elif left_reflection < thresholdLeft and right_reflection < thresholdRight:
                # Both sensors see black = Intersection. Will only see two black from the outer to inner turn.
                self.robot.drive(self.DRIVE_SPEED, -15)
                t.sleep(2)
            else:  # Both sensors see white
                self.robot.drive(self.DRIVE_SPEED, 0)  # Go straight

            wait(10)

# Create an instance of the robot
robot = RallyRobot(4, 28, 7, 28)

# Start robot
robot.followLine()