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

        self.motor0Params = self.interface.MotorAngleControllerParameters()
        self.motor0Params.maxRotationAcceleration = 6.0
        self.motor0Params.maxRotationSpeed = 12.0
        self.motor0Params.feedForwardGain = 255/20.0
        self.motor0Params.minPWM = 18.0
        self.motor0Params.pidParameters.minOutput = -255
        self.motor0Params.pidParameters.maxOutput = 255
        self.motor0Params.pidParameters.k_p = 700.0 #250.0
        self.motor0Params.pidParameters.k_i = 950.0 #200.0
        self.motor0Params.pidParameters.K_d = 20.0 #100.0
        self.interface.setMotorAngleControllerParameters(motors[0],self.motor0Params)

        self.motor1Params = self.interface.MotorAngleControllerParameters()
        self.motor1Params.maxRotationAcceleration = 6.0
        self.motor1Params.maxRotationSpeed = 12.0
        self.motor1Params.feedForwardGain = 255/20.0
        self.motor1Params.minPWM = 18.0
        self.motor1Params.pidParameters.minOutput = -255
        self.motor1Params.pidParameters.maxOutput = 255
        self.motor1Params.pidParameters.k_p = 700.0 #250.0
        self.motor1Params.pidParameters.k_i = 950.0 #200.0
        self.motor1Params.pidParameters.K_d = 20.0 #100.0
        self.interface.setMotorAngleControllerParameters(motors[1],self.motor1Params)

    def reach_target_angles(self, angle1, angle2):
        self.interface.increaseMotorAngleReferences(self.motors,[angle1,angle2])
        previous_total_angle_difference = 1000 #random value
        count = 0
        while not self.interface.motorAngleReferencesReached(self.motors) :
            motorAngles = self.interface.getMotorAngles(self.motors)
            referenceAngles = self.interface.getMotorAngleReferences(self.motors)
            if motorAngles :
                print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                print "Want to reach ", referenceAngles[0], ", ", referenceAngles[1]
                total_angle_difference = abs(motorAngles[0][0] - referenceAngles[0]) + abs(motorAngles[1][0] - referenceAngles[1])
                if previous_total_angle_difference == total_angle_difference:
                    count += 1
                else:
                    count = 0
                if count >= 30:
                    break
                previous_total_angle_difference = total_angle_difference
            time.sleep(0.1)

    def move_forward(self, angle):
        self.reach_target_angles(angle, angle)

    def move_forward_cm(self, distance):
        self.move_forward((1.0/self.wheelradius)*distance)

    def distance_to_wheel_angle(self, distance):
        return distance/self.wheelradius

    def turn(self, side, angle):
        if side == "left":
            self.reach_target_angles(-self.distance_to_wheel_angle(self.robotwidth*angle/2), self.distance_to_wheel_angle(self.robotwidth*angle/2))
        elif side == "right":
            self.reach_target_angles(self.distance_to_wheel_angle(self.robotwidth*angle/2), -self.distance_to_wheel_angle(self.robotwidth*angle/2))
        else:
            print("Unknown command. Expected 'left' or 'right' in turn method")

    def setP(self, p):
        self.motor1Params.pidParameters.k_p = p #250.0
        self.interface.setMotorAngleControllerParameters(self.motors[1],self.motor1Params)


# for r in range(4):

# rad = float(input("Enter a rad value: ")) #2.7
# 41cm squares with 2.7, 15.74
robot = MyRobot([0,1], 2.74, 15.97)
robot.interface.startLogging('logger.txt')
# robot.move_forward(4 * math.pi)
# robot.setP(p + 50*r)
# robot.move_forward_cm(40.0)

# for i in range(3):
for i in range(4):
    robot.move_forward_cm(40.0)
    robot.turn('right', math.pi/2)

robot.interface.stopLogging()
robot.interface.terminate()
