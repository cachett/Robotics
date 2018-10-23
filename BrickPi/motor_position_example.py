import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = interface.MotorAngleControllerParameters()
motorParams.maxRotationAcceleration = 6.0
motorParams.maxRotationSpeed = 12.0
motorParams.feedForwardGain = 255/20.0#To modify
motorParams.minPWM = 18.0 #To modify
motorParams.pidParameters.minOutput = -255#To modify
motorParams.pidParameters.maxOutput = 255#To modify
motorParams.pidParameters.k_p = 100.0#To modify
motorParams.pidParameters.k_i = 100.0#To modify
motorParams.pidParameters.k_d = 100.0#To modify

interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

interface.startLogging("logger.txt")

while True:
	angle = float(input("Enter a angle to rotate (in radians): "))

	interface.increaseMotorAngleReferences(motors,[angle,angle])

	while not interface.motorAngleReferencesReached(motors) :
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print ("Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0])
		time.sleep(0.1)

	print ("Destination reached!")

interface.stopLogging()
interface.terminate()
