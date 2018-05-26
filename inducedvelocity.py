#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Şan Kılkış"

from globs import *
import numpy as np
import matplotlib.pyplot as plt

v_i_hover = sqrt(W/(2*rho*pi*(R**2)))

V = np.linspace(1, 200, 100)

V_bar = V/v_i_hover

v_i_highspeed = 1 / V_bar

fig = plt.figure('MediumLiftCoeficient')
plt.style.use('ggplot')
plt.plot(V_bar, v_i_highspeed, label='Data')
plt.show()


print v_i_hover
