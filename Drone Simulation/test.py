import numpy as np

tick = 10
t = tick * 0.1
area_radius = 5
tau = 1

t_squ = t*t
pi_squ4 = 4 * (np.pi * np.pi)

x_s = (area_radius * t_squ * np.cos(2*np.pi*t)) #/ pi_squ4
y_s = (area_radius * t_squ * np.sin(2*np.pi*t)) #/ pi_squ4

x_s_squ = x_s*x_s
y_s_squ = y_s*y_s

h_t = (tau * (x_s_squ + y_s_squ)) / (area_radius*area_radius)
h_t_squ = h_t*h_t

r_s = (tau * (h_t_squ * h_t_squ)) / (area_radius*area_radius)
r_s_squ = r_s * r_s

x_t = (area_radius * r_s_squ * np.cos(2*np.pi*r_s)) #/ pi_squ4
y_t = (area_radius * r_s_squ * np.sin(2*np.pi*r_s)) #/ pi_squ4

print(r_s)
print (x_t, y_t)