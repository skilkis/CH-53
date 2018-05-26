#  This script will Perform the calculations of parts 4 5 and 6 of assignment 1, for AE4314.

__author__ = ['Nelson Johnson']

'''ALL UNITS IN THIS SCRIPT ARE SI'''

from math import *
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.optimize import minimize
import numpy as np

# Inputs
g = 9.81        # Gravitational Acceleration [m/s**2]

m = 19051       # mass [kg]
W = m*g         #  Weight [N]
D = 21.95       #  Rotor Diameter
R = D/2.0   # Rotor Radius [m]
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
print 'Disk Loading =', DL, '[N/m^2]'

#  Calc Tip Mach Number
#  TODO Figure this and C_dp bar out pg 30 reader.
a_inf = sqrt(1.4*287.1*T_inf)       #  Freestream Speed of Sound
V_t = omega*R
M_t = (V_t+V_cr)/a_inf                     #  Tip Mach number

#  Equivalent Flat Plate Area
sum_cds = 4.23                       #  This is the Equivalent Flat Plate Area estimated from pg 52 of reader

#  Question 4-1: Hover induced velocity using ACT theory
v_i_ACT = sqrt(W / (2 * pi * rho * R ** 2))
print 'Question 4-1: Hover ACT Induced Velocity = ',v_i_ACT, 'm/s'

#  TODO  Question 4-2:

#  TODO Question 4-3:


#  Question 5-4: Calculate the Helicopter Ideal Power in ACT theory
P_i_ACT = W * v_i_ACT                   #  Ideal Power ACT [W]
print 'Question 5-4: ACT Ideal Hover Power = ', P_i_ACT, 'W'

#  Question 5-5: Calculate the hover power in ACT theory
P_hov_ACT = P_i_ACT / FM
PL_ACT = W / P_hov_ACT
print 'Question 5-5: ACT Hover Power = ', P_hov_ACT, 'W'
print 'Hover ACT Power Loading = ', PL_ACT, 'N/W'


#  Question 5-6: Hover Power BEM theory
P_i_BEM = k*W*v_i_ACT
P_p_BEM = ((psi*C_dp)/8.0)*((rho*(omega*R)**3)*pi*R**2)
P_hov_BEM = P_i_BEM+P_p_BEM
PL_BEM = W / P_hov_BEM
print 'Tip Mach Number =', M_t
print 'Question 5-6: Hover BEM Power = ', P_hov_BEM, 'W'
print 'Hover BEM Power Loading = ', PL_BEM, 'N/W'


#  Question 5-7: FWD Flight Rotor Power using parasite, induced and profile drag pwr. These equations are from pg 48
#  of the reader.
V = V_cr                        #  Flight Velocity
u = V/(omega*R)                 #  Tip Speed Ratio
v_i_hispeed = W/(2*pi*(R**2)*rho*V)                                             #  High speed induuced velocity
V_bar = V/v_i_hispeed              #  Non-Dimensionalized Velocity
v_i_bar = 1/V_bar
#P_i_fwd = k*W*(sqrt((-(V_bar**2)/2.0)+sqrt(((V_bar**4)/4)+1)))*v_i_ACT         #  Induced PWR FWD Flight (low speed)
P_i_fwd = k*W*v_i_bar*v_i_hispeed                                                 #  Induced PWR FWD Flight (high speed)
P_p_fwd = ((psi*C_dp)/8.0)*rho*((omega*R)**3)*pi*(R**2)*(1+(4.65*(u**2)))       #  Profile PWR FWD Flight
P_par_fwd = sum_cds*0.5*rho*(V**3)                                              #  Parasite Drag PWR FWD Flight
P_t0 = P_i_fwd + P_p_fwd+ P_par_fwd

print 'Question 5-7: The Total Power If Forward Flight at V=', V, 'm/s is ', P_t0, 'W'

C_T = W / (rho * pi * (R ** 2) * ((omega * R) ** 2))

C_L_bar = (6.6 * C_T) / psi

print 'Trust Coefficient is ', C_T
print 'Medium Lift Coefficient is ', C_L_bar


# Question 5-9: Plot Power Required components from 5-7 as a function of velocity
V_loop = []
u_loop = []
V_bar_loop = []
P_i_fwd_loop = []
P_p_fwd_loop = []
P_par_fwd_loop = []
P_fwd_loop = []

#  TODO finalize max speed in this loop.
for i in range(0,200,1):
    V_loop.append(i)
    u = V_loop[i] / (omega * R)
    u_loop.append(u)

    V_bar = V_loop[i] / v_i_ACT
    V_bar_loop.append(V_bar)

    P_i_fwd = k * W * (sqrt((-(V_bar_loop[i] ** 2) / 2.0) + sqrt(((V_bar_loop[i] ** 4) / 4) + 1))) * v_i_ACT
    P_i_fwd_loop.append(P_i_fwd)

    P_p_fwd = ((psi * C_dp) / 8.0) * rho * ((omega * R) ** 3) * pi * (R ** 2) * (1 + (4.65 * (u_loop[i] ** 2)))
    P_p_fwd_loop.append(P_p_fwd)

    P_par_fwd = sum_cds * 0.5 * rho * (V_loop[i] ** 3)
    P_par_fwd_loop.append(P_par_fwd)

    P_fwd = P_i_fwd + P_p_fwd + P_par_fwd
    P_fwd_loop.append(P_fwd)


#  Question 5-8: Calculate Tail Rotor Power Using BEM theory
#  This is done for FWD Flight for the range of flight velocities in 5-9
T_tr_lst = []
v_i_tr_lst = []
P_i_tr_lst = []
P_p_tr_lst = []
P_tr = []
#  Tail rotor Profile drag = constant with speed!
P_p_tr = ((psi_tr*C_dp)/8.0)*rho*((omega_tr*R_tr)**3)*pi*R_tr**2
for i in range(0,len(V_loop),1):
    T_tr = P_fwd_loop[i]/(omega*l_tr)
    T_tr_lst.append(T_tr)

    v_i_tr = sqrt(T_tr/(2*pi*rho*(R_tr**2)))
    v_i_tr_lst.append(v_i_tr)

    P_i_tr = 1.1*k_tr*T_tr*v_i_tr
    P_i_tr_lst.append(P_i_tr)

    P_tr.append(P_i_tr_lst[i]+P_p_tr)

print 'Question 5-8: Tail Rotor Power using BEM is shown in plot as a function of V'
print 'Question 5-9: Power Required components are shown in plot as a function of V'

# TODO Question 6
#  V_emax = V(P_min)
#  V_Rmax = V(????)

P_misc_frac = 1.07 # W4_L08_P2dem_forward flight power.pptx.pdf
P_tot = [(P_fwd_loop[V] + P_tr[V])*P_misc_frac for V in range(0, len(V_loop))]

# P_tot_func = interp1d(V_loop, P_tot, 'linear', fill_value='extrapolate')
# val = minimize(P_tot_func)
# print val
error = []
slope = []
for i in range(0, len(V_loop)-1):
    tangent = P_tot[i+1]/V_loop[i+1]
    local_tangent = (P_tot[i+1]-P_tot[i])/(V_loop[i+1]-V_loop[i])
    error.append(abs(tangent - local_tangent))
    slope.append(tangent)

idx_max_range = error.index(min(error))
V_max_range = V_loop[idx_max_range]

idx_max_endurance = P_tot.index(min(P_tot))

V_max_endurance = V_loop[idx_max_endurance]





#  Plot the power components :)
plt.style.use('ggplot')
plt.plot(V_loop, P_i_fwd_loop, label='Rotor Induced Power')
plt.plot(V_loop, P_p_fwd_loop, label='Rotor Profile Power')
plt.plot(V_loop, P_par_fwd_loop, label='Rotor Parasitic Power')
plt.plot(V_loop, P_tr, label='Tail Rotor (induced and profile) Power')
plt.plot(V_loop, P_tot, label='Total Power', linestyle='-.')
plt.plot([0, V_max_range, V_loop[-1]], [0, P_tot[idx_max_range], V_loop[-1]*slope[idx_max_endurance]],
         linestyle='-.',
         color='k')
plt.plot(V_max_range, P_tot[idx_max_range],
         marker='o',
         markerfacecolor='white',
         markeredgecolor='black', markeredgewidth=1,
         linewidth=0,
         label=r'$V_{R_{\mathrm{max}}} = %0.1f$ [m/s]' % V_max_range)
plt.plot(V_max_endurance, P_tot[idx_max_endurance],
         marker='o',
         markerfacecolor='grey',
         markeredgecolor='black', markeredgewidth=1,
         linewidth=0,
         label=r'$V_{E_{\mathrm{max}}} = %0.1f$ [m/s]' % V_max_endurance)

plt.title('Forward Flight Drag Components')
plt.xlabel('Flight Speed [m/s]')
plt.ylabel('Power [W]')
plt.legend()
plt.show()



print 'Blade Solidity', psi
