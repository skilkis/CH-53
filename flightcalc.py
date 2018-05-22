#  This script will Perform the calculations of parts 4 5 and 6 of assignment 1, for AE4314.
#  The authors are Nelson Johnson and San Kilkis.

'''ALL UNITS IN THIS SCRIPT ARE SI'''

from math import *


# Inputs
g = 9.81        # Gravitational Acceleration [m/s**2]
rho = 1.225     # Density kg/m^3
W = 19051       # Weight [kg]
D = 21.95       #  Rotor Diameter
R = D/2.0   # Rotor Radius [m]

FM = 0.7        #  Assumed Figure of Merit.

#  Question 4-1: Hover induced velocity using ACT theory
v_i = sqrt(W/(2*pi*rho*R**2))

print '4-1: Hover ACT Induced Velocity = ',v_i, 'm/s'

#  TODO  Question 4-2:

#  TODO Question 4-3:


#  Question 5-4: Calculate the Helicopter Ideal Power
P_i = W*g*v_i                   #  Ideal Power ACT [W]
print 'Question 5-4: Hover ACT Ideal Power = ', P_i, 'W'

#  Question 5-5: Calculate the hover power in ACT theory
P_hov = P_i / FM
PL_act = (W*g)/P_hov
print 'Question 5-4: Hover ACT Power = ', P_hov, 'W'
print 'Hover ACT Power Loading = ', PL_act, 'N/W'





