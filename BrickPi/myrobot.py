import brickpi
import time
import math
import random

class MyRobot:

    def __init__(self, motors, wheelradius, robotwidth):

        #Initialization of robot Interface
        self.interface = brickpi.Interface()
        self.interface.initialize()
        self.wheelradius = wheelradius
        self.robotwidth = robotwidth
        self.motors = motors
        self.interface.motorEnable(motors[0])
        self.interface.motorEnable(motors[1])

        #TOUCH SENSOR
        self.touch_port = [1, 3] #port 1 = left, port 3 = right
        self.interface.sensorEnable(self.touch_port[0], brickpi.SensorType.SENSOR_TOUCH)
        self.interface.sensorEnable(self.touch_port[1], brickpi.SensorType.SENSOR_TOUCH)


        #PID's Parameters motor 0
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

        #PID's Parameters motor 1
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

        #Wait for the angle to be reached
        while not self.interface.motorAngleReferencesReached(self.motors) :
            time.sleep(0.01)

    def move_forward(self, angle):
        self.reach_target_angles(angle, angle)

    def move_forward_cm(self, distance):
        self.move_forward((1.0/self.wheelradius)*distance)

    def move_forward_cm_speed(self, distance, speed):
        """
        Speed has to be provided in meter per second
        """
        check_frequency = 40
        discretised_distance = speed/check_frequency
        while distance > 0:
            self.move_forward((1.0/self.wheelradius)*discretised_distance)
            distance -= discretised_distance
            time.sleep(1.0/check_frequency)

    def distance_to_wheel_angle(self, distance):
        return distance/self.wheelradius

    def turn(self, side, angle):
        if side == "left":
            self.reach_target_angles(-self.distance_to_wheel_angle(self.robotwidth*angle/2), self.distance_to_wheel_angle(self.robotwidth*angle/2))
        elif side == "right":
            self.reach_target_angles(self.distance_to_wheel_angle(self.robotwidth*angle/2), -self.distance_to_wheel_angle(self.robotwidth*angle/2))
        else:
            print("Unknown command. Expected 'left' or 'right' in turn method")


    def avoid_obstacle(self, touched_0, touched_1):
        self.move_forward_cm(-5)
        direction = ['left', 'right']

        if touched_0 and touched_1:
            #obstacle in front of us
            self.turn(direction[random.randint(0,1)], random.randint(90,175)*math.pi/180)
        elif touched_0:
            #obstacle to the left
            self.turn(direction[1], random.randint(90,175)*math.pi/180)

        elif touched_1:
            #obstacle to the right
            self.turn(direction[0], random.randint(90,175)*math.pi/180)

    def square(self, length):
        for _ in range(4):
            robot.move_forward_cm(length)
            robot.turn('right', math.pi/2)

    def move_forward_avoid_obstacles(self):
        """
        Continueously move foward until it reaches an obstacle. Then avoid it and keep moving forward.
        """
        angle1 = 10
        angle2 = 10
        while True:
            state_touch_sensor_0 = self.interface.getSensorValue(self.touch_port[0])
            state_touch_sensor_1 = self.interface.getSensorValue(self.touch_port[1])
            if state_touch_sensor_0 and state_touch_sensor_1:
                #handle obstacle
                touched_0 = state_touch_sensor_0[0]
                touched_1 = state_touch_sensor_1[0]
                if touched_0 or touched_1:
                    self.avoid_obstacle(touched_0, touched_1)
                else:
                    #keep moving foward
                    angle1 += 1
                    angle2 += 1
                    self.interface.increaseMotorAngleReferences(self.motors,[angle1,angle2])


def main():
    robot = MyRobot([0,1], 2.74, 15.97)
    robot.interface.startLogging('logger.txt')

    # robot.move_forward_avoid_obstacles()
    robot.move_forward_cm_speed(100, 0.25)

    robot.interface.stopLogging()
    robot.interface.terminate()

if __name__ == '__main__':
    main()
