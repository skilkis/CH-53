# -*- coding: utf-8 -*-
# from components import Battery
""" A file containing all relevant mass moment of inertias as functions """

class Inertia(object):

    def __init__(self, mass):
        self.mass = mass

    def cylinder(self, radius, length):
        I_yy = 1.0/12.0 * self.mass * (3 * (radius ** 2) + length
        I_zz = I_yy
        I_xx = 2.0

