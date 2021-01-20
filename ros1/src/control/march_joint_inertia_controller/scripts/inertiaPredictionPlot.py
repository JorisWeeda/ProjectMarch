"""Script to plot the inertia based on the gait"""

import numpy as np
import matplotlib.pyplot as plt

I1 = 2.09e-4 #inertia of unloaded joint in kg*m^2
lever_length = 0.5
lever_width = 0.075
lever_thick = 0.01
alu_dens = 2700
m1 = lever_length * lever_width * lever_thick * alu_dens #mass of lever arm in kg
m2 = 1 #mass of traveling weight in kg
r1 = 0.5 #total length of lever arm in m
r2 = r1/2 #half the length of the lever arm in m
g = 9.81 #gravitational accelerationt

#simulation charactheristics
t_total = 5 #total time in seconds
accel = np.pi / (2 * (t_total / 4) ** 2)
n = 1000 #timesteps
h = t_total/n #timestep for eulers integration algorithm

#vectors
time_vector = np.zeros(n)  #time vector
accel_vector = np.zeros(n) #vector with non-zero starting values
inertia_vector = np.zeros(n) #vector with non-zero starting values
vel_vector = np.zeros(n) #vector with starting value zero
angle_vector = np.zeros(n) #vector with starting value zero


#filling the arrays
for i in range(n):
    time_vector[i] = (t_total/n)*i
    if time_vector[i] < 1.25 or time_vector[i] > 3.75:
        accel_vector[i] = accel
    else:
        accel_vector[i] = - accel

for i in range(1,n):
    vel_vector[i] = vel_vector[i-1] + h * accel_vector[i-1]
    angle_vector[i] = angle_vector[i-1] + h * vel_vector[i-1]

for i in range(n):
    inertia_vector[i] = accel_vector[i] * (I1 + m1 * r1 ** 2 / 3 + m2 * r1 ** 2) \
                        - m1 * g * r2 * np.cos(angle_vector[i]) \
                        - m2 * g * r2

plt.plot(time_vector, angle_vector)
plt.show()

plt.plot(time_vector, inertia_vector)
plt.show()

plt.plot(angle_vector, inertia_vector)
plt.show()
