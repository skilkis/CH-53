# -*- coding: utf-8 -*-
# from components import Battery
""" This file contains the class definition used to estimate the Mass Moment of Inertia of the CH53 Helicopter """

__author__ = ["San Kilkis"]

from globs import Constants
from masses import ComponentWeights
from cla_regression import LiftGradient

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt


class Test(Constants):

    @property
    def weights(self):
        """

        :return: Class definition of
        """
        return ComponentWeights()

    @property
    def inerta_blade(self):
        """ Single blade Mass Moment of Inertia about the flapping hinge assuming that the blade length runs from
        hub-center to tip.

        :return: Mass Moment of Inertia in SI kilogram meter squared [kg m^2]
        """
        return (1.0/3.0) * (self.weights.kg_to_lbs(self.weights.W_2A, power=-1) / self.weights.n) * self.R**2

    @property
    def lift_gradient(self):
        return LiftGradient().gradient


    @property
    def lock_number(self):
        return (self.rho * self.lift_gradient * self.c * (self.R ** 4)) / self.inerta_blade


if __name__ == '__main__':
    x = Test()
    print x.lock_number



# # def func_statespace(Y, t):
# #     zeta_
# #     return [Y[1], -2 * Y[0]-Y[1]]
#
# a_t = np.linspace(0, 25.0, 1000)
#
# def main(a_t):
#     sol = integrate.odeint(func_statespace, [-1, 0], a_t)
#     beta = [beta for beta_prime, beta in sol]
#     return beta
#
# fig = plt.figure('LALALAL')
# plt.plot(a_t, main(a_t))
# plt.show()


# class PlaceHolder(Constants):
#
#     def
#
# if __name__ == '__main__':
#     obj = PlaceHolder()
#     print obj.R

# def flapping_hover(theta0=8.0):
#
#     def flapping_hover(self):