#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import sys

##/////////////////////////////////////////////////// HANDLE DATA FROM LOG FILE \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\##

filename = sys.argv[1]
fd = open(filename, "r")
array = fd.readlines()
final_array = []

for line in array:
    line_array = line[:-1].split('\t')[:-1]
    final_line = []

    for elt in line_array:
        if elt == '':
            continue
        else:
            final_line.append(float(elt))

    if len(line_array) == 5:
        final_array.append(final_line)


##/////////////////////////////////////////////////// PLOT DATA \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\##

# print(final_array)
time = [elt[0] for elt in final_array]
motor0_angle = [elt[1] for elt in final_array]
motor0_angle_goal = [elt[2] for elt in final_array]
motor1_angle = [elt[3] for elt in final_array]
motor1_angle_goal = [elt[4] for elt in final_array]
motor0_angle_error = [elt[2] - elt[1] for elt in final_array]
motor1_angle_error = [elt[4] - elt[3] for elt in final_array]

# Plot motor angle vs reference angle
plt.figure(0)
# plt.plot(time, motor0_angle, label = "motor0 angle")
# plt.plot(time, motor0_angle_goal, label="motor0 reference angle")
plt.plot(time, motor1_angle, label="motor1 angle")
plt.plot(time, motor1_angle_goal, label="motor1 reference angle")
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()
plt.title('Evolution of angle of motors over time')
plt.grid(True)

# Plot angle error for motor0
plt.figure(1)
plt.plot(time, motor0_angle_error, label = "motor0 Angle Error")
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()
plt.title('Error of motor0 angle over time')
plt.grid(True)

# Plot angle error for motor0
plt.figure(2)
plt.plot(time, motor1_angle_error, label = "motor1 Angle Error")
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()
plt.title('Error of motor0 angle over time')
plt.grid(True)

plt.show()
