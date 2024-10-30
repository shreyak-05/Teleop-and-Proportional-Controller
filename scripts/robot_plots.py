#!/usr/bin/env python3

# Importing relevant packages
import numpy as np
import matplotlib.pyplot as plt
import math

# Loading in data files saved from most recent scenarios
#time_data_s1 = np.load('time_data_s1.npy')
x_position = np.load('x_position.npy')

#time_data_s2 = np.load('time_data_s2.npy')
y_position = np.load('y_position.npy')


# Converting data into floats
#time_data_s1 = time_data_s1.astype(float)
x_position = x_position.astype(float)

#time_data_s2 = time_data_s2.astype(float)
y_position = y_position.astype(float)



# Plotting Velocity Vs. Time and Position Vs. Time for each Scenario of the Turtlebot3 Simulation
# Scenario 1
plt.figure(1)
plt.plot(y_position, x_position)

plt.title('Robot Position')
plt.xlabel('Y Axis')
plt.ylabel('X Axis')




plt.show()