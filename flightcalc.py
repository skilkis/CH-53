#  This script will Perform the calculations of parts 4 5 and 6 of assignment 1, for AE4314.

__author__ = ['Nelson Johnson']

'''ALL UNITS IN THIS SCRIPT ARE SI'''

import matplotlib.pyplot as plt
from globs import *
from inducedvelocity import v_i_func, v_i_tr_func
import os

_working_dir = os.getcwd()  # Added for save functionality

#  Question 4-1: Hover induced velocity using ACT theory
v_i_ACT = sqrt(W / (2 * pi * rho * R ** 2))
print 'Question 4-1: Hover ACT Induced Velocity = ',v_i_ACT, 'm/s'

#  TODO  Question 4-2:

#  TODO Question 4-3:


#  Question 5-4: Calculate the Helicopter Ideal Power in ACT theory
P_i_ACT = W * v_i_ACT  # Ideal Power ACT [W]
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
print 'Question 5-6: Hover BEM Power = ', P_hov_BEM, 'W'
print 'Hover BEM Power Loading = ', PL_BEM, 'N/W'


#  Question 5-7: FWD Flight Rotor Power using parasite, induced and profile drag pwr. These equations are from pg 48
#  of the reader.
V = V_cr                        #  Flight Velocity
u = V/(omega*R)                 #  Tip Speed Ratio
v_i_hispeed = W/(2*pi*(R**2)*rho*V)                                             #  High speed induuced velocity
V_bar = V/v_i_hispeed             #  Non-Dimensionalized Velocity
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

for i in range(0, 200, 1):
    V_loop.append(i)
    u = V_loop[i] / (omega * R)
    u_loop.append(u)

    P_i_fwd = k * W * v_i_func(i)
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
for i in range(0, len(V_loop)):
    T_tr = P_fwd_loop[i]/(omega*l_tr)
    T_tr_lst.append(T_tr)

    v_i_hover_tr = sqrt(T_tr/(2*pi*rho*(R_tr**2)))
    V_bar_tr = V_loop[i]/v_i_hover_tr
    v_i_tr = v_i_tr_func(V_bar_tr) * v_i_hover_tr

    P_i_tr = 1.1*k_tr*T_tr*v_i_tr
    P_i_tr_lst.append(P_i_tr)

    mu_tr = V_loop[i]/(omega_tr*R_tr)
    P_p_tr = ((psi_tr*C_dp)/8.0)*rho*((omega_tr*R_tr)**3)*pi*R_tr**2 * (1+4.65*(mu_tr**2))

    P_tr.append(P_i_tr_lst[i]+P_p_tr)


fig = plt.figure('TailRotorPower', figsize=(8, 3), dpi=80)
plt.style.use('ggplot')
plt.plot(V_loop, [pwr / 1000 for pwr in P_tr])
plt.title('Tail Rotor Power as a Function of Forward Flight Velocity')
plt.xlabel(r'True Airspeed $V_{\mathrm{TAS}}$ [m/s]')
plt.ylabel('Power [kW]')
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

print 'Question 5-8: Tail Rotor Power using BEM is shown in plot as a function of V'
print 'Question 5-9: Power Required components are shown in plot as a function of V'

# TODO Question 6
#  V_emax = V(P_min)
#  V_Rmax = V(????)

P_misc_frac = 1.07  # W4_L08_P2dem_forward flight power.pptx.pdf
P_tot = [(P_fwd_loop[V] + P_tr[V])*P_misc_frac for V in range(0, len(V_loop))]

fig = plt.figure('PowervsAirspeed')
plt.style.use('ggplot')
plt.plot(V_loop, [pwr / 1000 for pwr in P_i_fwd_loop], label='Rotor Induced Power')
plt.plot(V_loop, [pwr / 1000 for pwr in P_p_fwd_loop], label='Rotor Profile Power')
plt.plot(V_loop, [pwr / 1000 for pwr in P_par_fwd_loop], label='Rotor Parasitic Power')
plt.plot(V_loop, [pwr / 1000 for pwr in P_tr], label='Tail Rotor (Induced and Profile) Power')
plt.plot(V_loop, [pwr / 1000 for pwr in P_tot], label='Total Power', linestyle='-.')

plt.title('Maximum Range and Maximum Endurance Airspeeds')
plt.xlabel(r'True Airspeed $V_{\mathrm{TAS}}$ [m/s]')
plt.ylabel('Power [kW]')
plt.legend()
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

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

fig = plt.figure('Speeds')
plt.style.use('ggplot')
plt.plot([0, V_max_range, V_loop[-1]], [0, P_tot[idx_max_range]/1000, V_loop[-1]*slope[idx_max_range]/1000],
         color='k', alpha=0.3)
plt.axhline(P_tot[idx_max_endurance]/1000, color='k', alpha=0.3)
plt.axvline(V_max_endurance, ymin=0, ymax=P_tot[idx_max_endurance], linestyle=':')
plt.axvline(V_max_range, ymin=0, ymax=P_tot[idx_max_range], linestyle=':')
plt.plot(V_loop, [pwr / 1000 for pwr in P_tot], label='Total Power', linewidth=2)
plt.plot(V_max_range, P_tot[idx_max_range]/1000,
         marker='o',
         markerfacecolor='white',
         markeredgecolor='black', markeredgewidth=1,
         linewidth=0,
         label=r'$V_{R_{\mathrm{max}}} = %0.1f$ [m/s]' % V_max_range)
plt.plot(V_max_endurance, P_tot[idx_max_endurance]/1000,
         marker='o',
         markerfacecolor='grey',
         markeredgecolor='black', markeredgewidth=1,
         linewidth=0,
         label=r'$V_{E_{\mathrm{max}}} = %0.1f$ [m/s]' % V_max_endurance)

plt.title('Total Forward Flight Power as a Function of Velocity')
plt.xlabel('Flight Speed [m/s]')
plt.ylabel('Power [kW]')
plt.legend()
plt.show()
fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

print 'Blade Solidity', psi
