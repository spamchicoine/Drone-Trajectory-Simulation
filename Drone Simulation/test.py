import numpy as np
import matplotlib.pyplot as plt


tick = 2
t = tick * 0.01
area_radius = 200
tau = 4

pi_squ = np.pi * np.pi
pi_squ4 = 4 * pi_squ

h_inv_t = 2*np.pi*np.sqrt(np.sqrt(t/tau))

x_t = (area_radius * (h_inv_t * h_inv_t) * np.cos(h_inv_t)) / pi_squ4
y_t = (area_radius * (h_inv_t * h_inv_t) * np.sin(h_inv_t)) / pi_squ4

theta = np.arctan(y_t/x_t)

current_arc_length = ((area_radius * (2 * ((theta * theta) + 4)**(3/2) - 16) )/ (24 * pi_squ))

print(h_inv_t)
print(theta)
print(current_arc_length)
print (x_t, y_t)