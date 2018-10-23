import brickpi
import time
import math

class MyRobot:

    def __init__(self, motors, wheelradius, robotwidth):
        self.interface = brickpi.Interface()
        self.interface.initialize()
        self.wheelradius = wheelradius
        self.robotwidth = robotwidth
        self.motors = motors
        self.interface.motorEnable(motors[0])
        self.interface.motorEnable(motors[1])
        self.motorParams = self.interface.MotorAngleControllerParameters()
        self.motorParams.maxRotationAcceleration = 8.0
        self.motorParams.maxRotationSpeed = 15.0
        self.motorParams.feedForwardGain = 255/20.0
        self.motorParams.minPWM = 18.0
        self.motorParams.pidParameters.minOutput = -255
        self.motorParams.pidParameters.maxOutput = 255
        self.motorParams.pidParameters.k_p = 100.0
        self.motorParams.pidParameters.k_i = 0.0
        self.motorParams.pidParameters.k_d = 0.0
        self.interface.setMotorAngleControllerParameters(motors[0],self.motorParams)
        self.interface.setMotorAngleControllerParameters(motors[1],self.motorParams)

    def specific_move(self, angle1, angle2):
        self.interface.increaseMotorAngleReferences(self.motors,[angle1,angle2])
        previousdiff = 1000 #random value
        count = 0
        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            referenceAngles = self.interface.getMotorAngleReferences(self.motors)
            if motorAngles :
                print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                print "Want to reach ", referenceAngles[0], ", ", referenceAngles[1]
                if previousdiff == abs(motorAngles[0][0] - referenceAngles[0]):
                    count += 1
                else:
                    count = 0
                if count >= 10:
                    break
                previousdiff = abs(motorAngles[0][0] - referenceAngles[0])
                previousMotorAngles0 = motorAngles[0][0]
                previousMotorAngles1 = motorAngles[1][0]
            time.sleep(0.1)

    def moveforward(self, angle):
        self.specific_move(angle, angle)

    def moveforwardcm(self, distance):
        self.moveforward((1.0/self.wheelradius)*distance)

    def right90deg(self):
        self.specific_move((self.robotwidth/(4.0*self.wheelradius))*math.pi, -(self.robotwidth/(4.0*self.wheelradius))*math.pi)


robot = MyRobot([0,1], 3.5, 17.8)
robot.moveforwardcm(40.0)
robot.right90deg()
robot.moveforwardcm(40.0)
robot.right90deg()
robot.moveforwardcm(40.0)
robot.right90deg()
robot.moveforwardcm(40.0)
robot.right90deg()
