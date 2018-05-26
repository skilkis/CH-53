#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Şan Kılkış"

from globs import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
import os
from math import sin, cos, asin, degrees

N = 100  # Fidelity of the arrays (Higher is better)
_working_dir = os.getcwd()  # Added for save functionality

# Calculation of the Non-Dimentionlized Velocity
v_i_hover = sqrt(W/(2*rho*pi*(R**2)))
V = np.linspace(1, 100, N)  # in [m/s]
V_bar = V/v_i_hover

# Calculation of the Induced Velocity for High-Speed
v_i_highspeed = 1 / V_bar

# Calculation of the Induced Velocity for Low-Speed utilizing a solver
v_i_lowspeed = []

for i in range(0, len(V_bar)):

    def func(x):
        return x**4 + ((V_bar[i]) ** 2 * (x ** 2)) - 1

    v_i_lowspeed.append(fsolve(func, x0=np.array([1])))

# Calculates the idx corresponding to where the High-Speed Approximation can be used:
error = [v_i_highspeed[i] - v_i_lowspeed[i] for i in range(0, len(V_bar))]

i = 0
while error > 0.001:
    error = v_i_highspeed[i] - v_i_lowspeed[i]
    if i > len(V_bar):
        raise ValueError('Warning no solution could be found for the current region')
    i = i + 1

idx = i

# Utilizing Numerical Methods to Solve w/o Assumptions:
D_par = sum_cds * rho * V**2
alpha_rad = [asin(D/W) for D in D_par]
alpha_deg = [degrees(rad) for rad in alpha_rad]

fig = plt.figure('AlphavsVelocity')
plt.style.use('ggplot')
plt.plot(V, alpha_deg)
plt.title(r'Disk Angle of Attack as a Function of Velocity')
plt.xlabel(r'Velocity [m/s]')
plt.ylabel(r'Disk Angle of Attack [deg]')
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

v_i = []
for i in range(0, len(V_bar)):
    D_par = sum_cds * rho * (V[i]**2)
    alpha_rad = asin(D_par / W)
    print degrees(alpha_rad)

    def func(x):
        return ((V_bar[i]*sin(alpha_rad)+abs(x))**2 + (V_bar[i]*cos(alpha_rad))**2) - (1/(abs(x)**2))

    v_i.append(abs(fsolve(func, x0=np.array([1]))[0]))


fig = plt.figure('InducedVelocityvsV')
plt.style.use('ggplot')
ax = fig.gca()

# Dashed Line to Indicate
plt.plot(V_bar[:idx+1], v_i_highspeed[:idx+1], linestyle='-.', color='k', alpha=0.5)

# Low-Speed Region
plt.plot(V_bar[:idx+1], v_i_lowspeed[:idx+1], label=r'Low-Speed')

# High-Speed Region
plt.plot(V_bar[idx:], v_i_highspeed[idx:], label='High-Speed')

# No Assumptions:
plt.plot(V_bar, v_i, label='Numerical Solution', marker='o')


plt.title('Induced Velocity as a Function of Forward Velocity')
plt.xlabel(r'Non-Dimensional Velocity $\overline{V}=\frac{\mu}{\sqrt{C_T/2}}$')
plt.ylabel(r'Non-Dimensional Induced Velocity $\overline{v}_i$')
plt.axis([0, 6, 0, 1.0])
plt.legend(loc='best')
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

