import matplotlib.pyplot as plt
import numpy as np

log_file = open('test.txt', 'r')
array = np.fromfile(log_file, dtype=int, sep='\t')
print(array)

# t = np.arange(1, len(s)+1, 1)
# print(max(s))
#
# plt.plot(t, s)
#
# plt.xlabel('Time')
# plt.ylabel('Angle')
# plt.title('Evolution of angle and reference angle over time')
# plt.grid(True)
# plt.savefig("angle.png")
# plt.show()
