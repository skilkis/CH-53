#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Provides all derived/non-derived inputs of the CH-53D Helicopter to be used later in performance calculations """

from math import pi, sqrt


class Constants(object):
    """ A class containing all global constants pertaining to the CH-53 Helicopter that are loaded as necessary """

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
    FM = 0.69       #  CH-53 Value of Figure of Merit from Sikorsky Archives
    V_cr_kmh = 278.4        #  Assumed Cruise speed used for tip mach no calculation [kmh]
    V_cr = V_cr_kmh/3.6     #  Cruise speed used for tip mach no calculation [m/s]

    k = 1.15        #  Assumed k factor for BEM rotor induced power
    k_tr = 1.4      #  Assumed k_tr for tail rotor BEM Power
    c_tr = 0.29     #  Tail Rotor Chord Length
    n_tr = 4.0      #  Number tail rotor Blades
    omega_tr_rpm = 699                  #  Tail Rotor Speed [rpm] from sikorsky archives
    omega_tr = (omega_tr_rpm*2*pi)/60.0 #  Tail Rotor Rotational Velocity [rad/s]

    #  Standard Sea Level Conditions
    rho = 1.225     # Density kg/m^3
    T_inf = 288.15  #  Freestream Temperature

    omega = 19.37    #  Main rotor Rotation rate [rad/s] from sikorsky archives

    # Total Engine Power
    P_e = 2926.872 * 2000 # Total Engine Power [W]

    #  Calculate Main Rotor Blade Solidity (psi)
    psi = (n*c)/(pi*R)

    #  Calculate Tail Rotor Blade Solidity
    psi_tr = (n_tr*c_tr)/(pi*R_tr)

    #  Calc Disk Loading
    DL = W/(pi*R**2)
    # print 'Disk Loading =', DL, '[N/m^2]'

    #  Calc Tip Mach Number
    a_inf = sqrt(1.4*287.1*T_inf)       #  Freestream Speed of Sound
    V_t = omega*R
    M_t = V_t/a_inf                     #  Tip Mach number

    C_dp = 0.0251     #  Blade average drag coefficient

    #  Equivalent Flat Plate Area
    sum_cds = 4.23                       #  This is the Equivalent Flat Plate Area estimated from pg 52 of reader


if __name__ == '__main__':
    obj = Constants()
