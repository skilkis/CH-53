#  This script will estimate the mass of the CH-53D components using the Weight Estimating Relations (WERs) derived in
#  NASA Parametric Study of Helicopter Aircraft Systems Costs and Weights by Beltramo and Morris.
#  The units are customary, and are given next to the variables.

from math import *

#  kg to lb conversion
f_lb = 2.2046
#  m to ft conversion
f_ft = 3.281
#  km to miles
f_mi = 0.6214

#  ---------------------------------------------------------------------------------------------------------------------
#  Inputs to WERs are below
#  Number of Main Rotor Blades
n = 6

#  Main Rotor Diameter and Radius [ft]
D = 21.95*f_ft
R = D/2.0

#  Tail Rotor Diameter and Radius [ft]
D_t = 4.9*f_ft
R_t = D/2.0

#  Gross Weight [lb]
W_g_kg = 19051  # [kg]
W_g = f_lb*W_g_kg

#  Empty Weight
W_oe_kg = 12020
W_oe = W_oe_kg*f_lb

#  Tandem Constant for Tail Structure TODO figure this out further?
K_t = 0

#  Main Rotor Blade chord (c) found from https://fas.org/man/dod-101/sys/ac/h-53.htm
c = 0.76*f_ft

#  Total Tail Surface Area (assuming this is the HT and VT Surface Area) [ft^2]
#  The VT and HT areas are derived from dimensioned engineering drawing.
S_tt = 8.41 * f_ft**2


#  Total Body Surface Area approximated with Square prism.
#  Fuselage Width and height [ft]
w_f_m = 2.68  # [m]
w_f_m = 2.5
w_f = w_f_m*f_ft
#  Fusselage Length
l_f_m = 19.07 # [m]
l_f_m = 12.54
l_f = l_f_m*f_ft
S_b = w_f*w_f*l_f

#  Nacelle Surface Area [ft^2]
#  TODO add proper r_n and l_n!!!!!!!!!!!!!!!
r_n = 0.75*f_ft
l_n = 2*f_ft
n_nac = 2.0
S_n = n_nac*pi*r_n**2*l_n

#  Engine Horsepower 2x GE T64-GE-413 [shp]
HP_e = 3925.0

#  Number of Gallons in Fuel Tank. Main Tank capacity is used [-]
G = 638.0

#  Range [mi]
r = 1000*f_mi

#  Number passengers and crew
N_p = 29.0
N_c = 2.0

#  ---------------------------------------------------------------------------------------------------------------------
#  Calculations
#  Blade Area [ft^2]
S_pl = n*c*R


#  ---------------------------------------------------------------------------------------------------------------------
#  Component Mass Estimations
#  1:  Wing Neglected


#  2: Total Rotor weight
W_2A = -88.742+6.403*S_pl   #  Blades Weight
W_2B = -105.943 + 5.761*S_pl
W_2 = W_2A+W_2B
WF_2 = W_2/W_oe*100


#  3: Tail Weight (Tail Rotor and Structure)
#  Tail Rotor Weight
#  W_3A = -29.916+0.0102*W_g
W_3A = W_g**1.352 / exp(8.327)
#  Tail Structure Weight
W_3B = -17.872+2.829*S_tt + K_t
W_3 = W_3A + W_3B
WF_3 = W_3/W_oe*100


#  4: Body Weight
W_4 = -269.023 + 2.356*S_b
WF_4 = W_4/W_oe*100


#  5: Landing Gear Weight
W_5 = -5.489 + 0.0342*W_g
WF_5 = W_5/W_oe*100


#  6: Nacelle Weight
W_6 = -64.779 + 2.401*S_n
WF_6 = W_6/W_oe*100


#  7: Propulsion System Weights engine = 2x GE T64-GE-413
#  Power Plant Weight 327 lb each found online
W_7A = 408.198 + 0.192*HP_e
#W_7A = 755.0*2.0

#  Drive System Weight
W_7B = -35.551+0.101*W_g

#  Fuel Tank Weights
W_7C = 10.974 + 0.790*G

#  Complete Power Plant weight
W_7 = W_7C+W_7B+W_7A
WF_7 = W_7/W_oe*100


#  8: Flight Controls Weight [lb]
W_8 = 62.025 + 0.0334*W_g
WF_8 = W_8/W_oe*100


#  9: Auxiliary Power Weight [lb]
W_9 = 157
WF_9 = W_9/W_oe*100


#  10: Instruments Weight [lb]
W_10 = 50.507 + 0.0267*HP_e
WF_10 = W_10/W_oe*100


#  11: Hydraulics Weight [lb]
W_11 = 15.89+0.00446*W_g
WF_11 = W_11/W_oe*100


#  12: Pneumatics Neglected


#  13: Electrical System Mass [lb]
#  W_13 = 139.947 + 0.234*S_b
W_13 = exp(0.903)*S_b**.733
WF_13 = W_13/W_oe*100

#  14: Avionics Weight [lb]
W_14 = -59.041 + 0.0175*W_g + 0.348*r
WF_14 = W_14/W_oe*100

#  15: Furnishings and Equipment [lb]
W_15 = -8.106 + 0.176*S_b + 20.456*(N_p + N_c)
WF_15 = W_15/W_oe*100

#  16 & 17: Air Conditioning and Anti-Icing
W_1617 = 28.844 + 0.0730*S_b
WF_1617 = W_1617/W_oe*100

#  18: Load and Handling [lb]
W_18 = -71.875 + 0.111*S_b + 3.489*(N_p + N_c)
WF_18 = W_18/W_oe*100

#  Payload [lb]
W_pl = 14000
WF_pl = W_pl/W_oe*100

#  Fuel Weight
V_f_m = 2415.1  # [l]
rho_f_m = .7750 # [kg/l]
W_f_kg = rho_f_m*V_f_m
W_f = W_f_kg*f_lb
WF_f = W_f/W_oe * 100


#  Sum of weight fractions percent of M_oe
WF_oe = (WF_2+WF_3+WF_4+WF_5+WF_6+WF_7+WF_8+WF_9+WF_10+WF_11+WF_13+WF_14+WF_15+WF_1617+WF_18)/100

#  Sum of weight fractions of W_g
WF_tot = (W_2+W_3+W_4+W_5+W_6+W_7+W_8+W_9+W_10+W_11+W_13+W_14+W_15+W_1617+W_18+W_pl+W_f)*100/W_g

print 'Gross Weight                 = ', W_g, 'lbs'
print '2A: Main Rotor Blades Weight = ', W_2A, 'lbs. Percent of W_oe = ', W_2A/W_oe*100
print '2B: Main Hub & Hinge Weight  = ', W_2B, 'lbs. Percent of W_oe = ', W_2B/W_oe*100
print '2: COMBINED Total Main Rotor Weight  = ', W_2, 'lbs. Percent of W_oe = ', WF_2
#  print '3A: Tail Rotor Weight        = ', W_3A, 'lbs. Percent of W_oe = ', W_3A/W_oe*100
#  print '3B: Tail Structure Weight    = ', W_3B, 'lbs. Percent of W_oe = ', W_3B/W_oe*100
print '3: Tail Weight               = ', W_3, 'lbs. Percent of W_oe = ', WF_3
print '4:  Body Weight              = ', W_4, 'lbs. Percent of W_oe = ', WF_4
print '5:  Landing Gear Weight      = ', W_5, 'lbs. Percent of W_oe = ', WF_5
print '6:  Nacelle Weight           = ', W_6, 'lbs. Percent of W_oe = ', WF_6
print '7A: Power Plant Weight       = ', W_7A, 'lbs. Percent of W_oe = ', W_7A/W_oe*100
print '7B: Drive System Weight      = ', W_7B, 'lbs. Percent of W_oe = ', W_7B/W_oe*100
print '7C: (Main) Fuel Tank Weight  = ', W_7C, 'lbs. Percent of W_oe = ', W_7C/W_oe*100
print '7:  Propulsion System Weight = ', W_7, 'lbs. Percent of W_oe = ', WF_7
print '8:  Flight Controls Weight   = ', W_8, 'lbs. Percent of W_oe = ', WF_8
print '9:  Auxiliary Power Weight   = ', W_9, 'lbs. Percent of W_oe = ', WF_9
print '10: Instruments Weight       = ', W_10, 'lbs. Percent of W_oe = ', WF_10
print '11: Hydraulics Weight        = ', W_11, 'lbs. Percent of W_oe = ', WF_11
print '12: Pneumatics Weight is Neglected'
print '13: Electronics Weight       = ', W_13, 'lbs. Percent of W_oe = ', WF_13
print '14: Avionics Weight          = ', W_14, 'lbs. Percent of W_oe = ', WF_14
print '15: Furnishings and Equipment Weight = ', W_15, 'lbs. Percent of W_oe = ', WF_15
print '16& 17: Air Conditioning and Anti-Icing Weight = ', W_1617, 'lbs. Percent of W_oe = ', WF_1617
print '18: Loads and Handling Weight = ', W_18, 'lbs. Percent of W_oe = ', WF_18
print 'xx: Payload Weight            = ', W_pl, 'lbs. Percent of W_oe = ', WF_pl
print 'xx: Fuel Weight               = ', W_f, 'lbs. Percent of W_oe = ', WF_f
print 'Sum of Component OE weight fraction', WF_oe
print 'Sum of Component Gross weight fraction', WF_tot


