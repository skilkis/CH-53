#  This script will Perform the calculations of parts 4 5 and 6 of assignment 1, for AE4314.
#  The authors are Nelson Johnson and San Kilkis.

'''ALL UNITS IN THIS SCRIPT ARE SI'''

from math import *


# Inputs
g = 9.81        # Gravitational Acceleration [m/s**2]
rho = 1.225     # Density kg/m^3
m = 19051       # mass [kg]
W = m*g         #  Weight [N]
D = 21.95       #  Rotor Diameter
R = D/2.0   # Rotor Radius [m]
n = 6           #  Number of Rotor Blades
c = 0.76        #  Blase chord [m]

FM = 0.7        #  Assumed Figure of Merit.

k = 1.15        #  Assumed k factor for BEM rotor induced power

#  TODO FIND CORRECT VALUE!!!!!!!!!!
omega = 19.37    #  Main rotor Rotation rate [rad/s]

#  TODO FIND THIS FINAL/APPLICABLE VALUE
C_dp = 0.01     #  Blase medium drag coefficient


#  Calculate Blade Solidity (psi)
psi = (n*c)/(pi*R)

#  Calc Disk Loading
DL = W/(pi*R**2)
print 'Disk Loading =', DL, '[N/m^2]'

#  Question 4-1: Hover induced velocity using ACT theory
v_i_ACT = sqrt(W / (2 * pi * rho * R ** 2))

print '4-1: Hover ACT Induced Velocity = ',v_i_ACT, 'm/s'

#  TODO  Question 4-2:

#  TODO Question 4-3:


#  Question 5-4: Calculate the Helicopter Ideal Power in ACT theory
P_i_ACT = W * v_i_ACT                   #  Ideal Power ACT [W]
print 'Question 5-4: Hover ACT Ideal Power = ', P_i_ACT, 'W'

#  Question 5-5: Calculate the hover power in ACT theory
P_hov_ACT = P_i_ACT / FM
PL_ACT = (W) / P_hov_ACT
print 'Question 5-4: Hover ACT Power = ', P_hov_ACT, 'W'
print 'Hover ACT Power Loading = ', PL_ACT, 'N/W'


#  Question 5-6: Hover Power BEM theory
P_i_BEM = k*W*v_i_ACT
P_p_BEM = ((psi*C_dp)/8.0)*((rho*(omega*R)**3)*pi*R**2)
P_hov_BEM = P_i_BEM+P_p_BEM
PL_BEM = (W) / P_hov_BEM
print P_i_BEM
print P_p_BEM
print 'Question 5-5: Hover BEM Power = ', P_hov_BEM, 'W'
print 'Hover BEM Power Loading = ', PL_BEM, 'N/W'


#  TODO Question 5-7

#  TODO Question 5-6: Calculate Tail Rotor Power Using BEM theory


