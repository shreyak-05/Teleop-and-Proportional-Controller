#!/usr/bin/env python3

# Importing relevant packages
import numpy as np
import matplotlib.pyplot as plt
import math


# This script plots the position (pose) of the robot as it travels from (0,0) ot (10, 10)


# Loading in data files saved from most recent scenarios

x_position = np.load('x_position.npy')
y_position = np.load('y_position.npy')


# Converting data into floats
x_position = x_position.astype(float)
y_position = y_position.astype(float)



# Plotting Pose of the Robot throughout the autonomous travel mode

plt.figure(1)
plt.plot(y_position, x_position)

plt.title('Robot Position')
plt.xlabel('Y Axis')
plt.ylabel('X Axis')

plt.show()