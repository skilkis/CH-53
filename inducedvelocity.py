#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = ["San Kilkis"]

from globs import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from scipy.interpolate import interp1d
import os
from math import sin, cos, asin, degrees
from collections import Iterable

N = 100  # Fidelity of the arrays (Higher is better)
_working_dir = os.getcwd()  # Added for save functionality

# Calculation of the Non-Dimentionlized Velocity
v_i_hover = sqrt(W/(2*rho*pi*(R**2)))

print v_i_hover

V = np.linspace(1, 100, N)  # in [m/s]
V_bar = V/v_i_hover

# Calculation of the Induced Velocity for High-Speed
v_i_highspeed = 1 / V_bar

# Calculation of the Induced Velocity for Low-Speed utilizing a solver
v_i_lowspeed = []

for i in range(0, len(V_bar)):

    def func(x):
        """ Defines a 4-th order equation v_i**4 + (V**2 * v_i**2) - 1 = 0

        :param x: Represents the Non-Dimensional Induced Velocity
        """
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
def alpha_disk(velocity):
    """ A function that returns the rotor disk angle of attack (AoA) per input flight velocity

    :param velocity: Forward flight velocity (gamma=0) in SI meter per second [m/s]
    :return: Rotor Disk Angle of Attack (AoA) in SI radians [rad]
    :rtype: list or float
    """
    D_par = sum_cds * rho * velocity ** 2  # Parasitic Drag Value
    if isinstance(velocity, Iterable) and type(velocity) is not str:
        alpha = [asin(D/W) for D in D_par]
    else:
        alpha = asin(D_par/W)
    return alpha


alpha_rad = alpha_disk(velocity=V)
alpha_deg = [degrees(rad) for rad in alpha_rad]

V_bar_cr = V_cr / v_i_hover
alpha_rad_cr = alpha_disk(velocity=V_cr)
alpha_deg_cr = degrees(alpha_rad_cr)

fig = plt.figure('AlphavsVelocity')
plt.style.use('ggplot')
plt.plot(V_bar, alpha_deg, label=r'Function')
plt.title(r'Disk Angle of Attack as a Function of Velocity')
plt.plot(V_bar_cr, alpha_deg_cr,
         marker='o',
         markerfacecolor='white',
         markeredgecolor='black', markeredgewidth=1,
         linewidth=0,
         label=r'Cruise Condition ($\overline{V}=$ %0.1f [-], $\alpha_d$ = %0.1f [deg])' % (V_bar_cr, alpha_deg_cr))
plt.xlabel(r'Non-Dimensional Velocity $\overline{V}=\frac{\mu}{\sqrt{C_T/2}}$')
plt.ylabel(r'Disk Angle of Attack [deg]')
plt.legend(loc='best')
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

v_i = []
for i in range(0, len(V_bar)):
    D_par = sum_cds * rho * (V[i]**2)
    alpha_rad = asin(D_par / W)

    def func(x):
        """ Defines a 2-th order equation function (V*sin(a) + v_i)**2 + (V*cos(a))**2 - (1/v_i)**2 = 0

        :param x: Represents the Non-Dimensional Induced Velocity
        """
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
# plt.plot(V_bar, v_i, label='Numerical Solution', marker='o')

# Obtaining Values for Haffner Diagram Check at Cruise Speed (Question 4-3)
haffner_x = V_bar_cr * cos(alpha_rad_cr)
haffner_y = V_bar_cr * sin(alpha_rad_cr)

print 'Haffner Diagram x,y = (%0.1f, %0.1f) for V_cr = %0.1f' % (haffner_x, haffner_y, V_bar_cr)

induced_velocity = [v * v_i_hover for v in v_i]
v_i_func = interp1d(V, induced_velocity, fill_value='extrapolate')

plt.title('Induced Velocity as a Function of Forward Velocity')
plt.xlabel(r'Non-Dimensional Velocity $\overline{V}=\frac{\mu}{\sqrt{C_T/2}}$')
plt.ylabel(r'Non-Dimensional Induced Velocity $\overline{v}_i$', )
plt.axvline(V_bar[idx], ymax=v_i_func(V[idx])/v_i_hover, linestyle=':', color='k', alpha=0.5)
plt.plot(V_bar_cr, v_i_func(V_cr)/v_i_hover,
         marker='o',
         markerfacecolor='white',
         markeredgecolor='black', markeredgewidth=1,
         linewidth=0,
         label=r'Cruise Condition ($\overline{V}_{cr}=$ %0.1f [-], $\overline{v}_i$ = %0.3f [deg])'
               % (V_bar_cr, v_i_func(V_cr)/v_i_hover))
plt.axis([0, 7, 0, 1.0])
plt.legend(loc='best')
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')


def v_i_tr_func(V_bar_tr):
    """ Utilizes ACT theory along w/ the approximation that the AoA of the Tail-Rotor Disk is zero for all speeds

    :param V_bar_tr: Non-Dimensional Tail Rotor Velocity
    :return: The Non-dimensional Induced Tail Rotor Velocity
    :rtype: float
    """

    def func(x):
        """ Defines a 4-th order equation v_i**4 + (V**2 * v_i**2) - 1 = 0

        :param x: Represents the Non-Dimensional Induced Velocity
        """
        return x**4 + ((V_bar_tr ** 2) * (x ** 2)) - 1

    return (fsolve(func, x0=np.array([1])))[0]

