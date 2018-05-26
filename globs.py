#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Provides all derived/non-derived inputs of the CH-53D Helicopter to be used later in performance calculations """

from math import pi, sqrt

# Inputs
g = 9.81        # Gravitational Acceleration [m/s**2]

m = 19051       # mass [kg]
W = m*g         #  Weight [N]
D = 21.95       #  Rotor Diameter
R = D/2.0       # Rotor Radius [m]
n = 6           #  Number of Rotor Blades
c = 0.76        #  Blase chord [m]
l_tr = 13.62    #  Tail Rotor Length (WRT main rotor) [m]
D_tr = 4.9      #  Tail Rotor Diameter [m]
R_tr = D_tr/2.0 #  Tail Rotor Radius [m]
FM = 0.7        #  Assumed Figure of Merit.
V_cr_kmh = 278.4        #  Assumed Cruise speed used for tip mach no calculation [kmh]
V_cr = V_cr_kmh/3.6     #  Cruise speed used for tip mach no calculation [m/s]


k = 1.15        #  Assumed k factor for BEM rotor induced power
k_tr = 1.4      #  Assumed k_tr for tail rotor BEM Power
c_tr = 0.29     #  Tail Rotor Chord Length
n_tr = 4.0      #  Number tail rotor Blades
omega_tr_rpm = 699                  #  Tail Rotor Speed [rpm] from sikorsky archives
omega_tr = (omega_tr_rpm*2*pi)/60.0 #  Tail Rotor Rotational Velocity [rad/s]

#  Altitude = 304.8m. This is input into a BSc Y1 Python script to get ISA temp and density.
rho = 1.1895    # Density kg/m^3
T_inf = 286.17  #  Freestream Temperature

omega = 19.37    #  Main rotor Rotation rate [rad/s] from sikorsky archives

#  TODO FIND THIS FINAL/APPLICABLE VALUE BY ESTIMATING C_L BAR (REGRESSION WITH W/piR**2). THEN CDP FROM PG 30.
C_dp = 0.025     #  Blade average drag coefficient


#  Calculate Main Rotor Blade Solidity (psi)
psi = (n*c)/(pi*R)

#  Calculate Tail Rotor Blade Solidity
psi_tr = (n_tr*c_tr)/(pi*R_tr)

#  Calc Disk Loading
DL = W/(pi*R**2)
# print 'Disk Loading =', DL, '[N/m^2]'

#  Calc Tip Mach Number
#  TODO Figure this and C_dp bar out pg 30 reader.
a_inf = sqrt(1.4*287.1*T_inf)       #  Freestream Speed of Sound
V_t = omega*R
M_t = (V_t+V_cr)/a_inf                     #  Tip Mach number

#  Equivalent Flat Plate Area
sum_cds = 4.23                       #  This is the Equivalent Flat Plate Area estimated from pg 52 of reader