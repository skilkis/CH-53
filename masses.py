#  This script will estimate the mass of the CH-53 components using the Weight Estimating Realations (WERs) derived in
#  NASA Parametric Study of Helicopter Aircraft Systems Costs and Weights by Beltramo and Morris.
#  The units are customary, and are given next to the variables.

#  kg to lb conversion
f_lb = 2.2046

#  m to ft conversion
f_ft = 3.281


#  Inputs to WERs are below

#  Number of Blades (n)
n = 6

#  Main Rotor Diameter and Radius
D = 21.95*f_ft
R = D/2.0

#  Blade chord (c) found from https://fas.org/man/dod-101/sys/ac/h-53.htm
c = 0.76*f_ft

#  Blade Area (S_pl)
S_pl = n*c*R

#  Gross Weight [lb]
W_g_kg = 19051
W_g = f_lb*W_g_kg



#  Component Mass Estimations

#  2: Total Rotor weight
W_2 = -194.685+12.164*S_pl

print W_g, W_2