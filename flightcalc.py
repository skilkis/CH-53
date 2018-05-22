#  This script will Perform the calculations of parts 4 5 and 6 of assignment 1, for AE4314.
#  The authors are Nelson Johnson and San Kilkis.
from math import *


# Inputs
rho = 1.225     # Density kg/m^3
W = 19051       # Weight [kg]
D = 21.95       #  Rotor Diameter
R = D/2.0   # Rotor Radius [m]


#  Question 4: Hover induced velocity using ACT theory

v_i = sqrt(W/(2*pi*rho*R**2))

print v_i