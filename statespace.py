#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

# http://python-control.readthedocs.io/en/latest/intro.html most likely this will be required

__author__ = ["Nelson Johnson", "San Kilkis"]

from globs import Constants, Attribute
from stabilityderivatives import StabilityDerivatives
from trim import Trim
import numpy as np
from control.matlab import *
# import scipy.signal as sgn
from math import sin, cos, radians, degrees
import matplotlib.pyplot as plt


class StateSpace(Constants):
    """ xxx

    :param u: xx
    :type u: xx
    """

    def __init__(self, initial_velocity=0.0, final_velocity=0.0, distance_to_helipad=100.0):
        self.initial_velocity = initial_velocity
        self.final_velocity = final_velocity
        self.distance_to_helipad = distance_to_helipad

    @Attribute
    def initial_trim_case(self):
        return Trim(self.initial_velocity)

    @Attribute
    def stability_derivatives(self):
        u = self.initial_trim_case.velocity * cos(self.initial_trim_case.fuselage_tilt)
        w = self.initial_trim_case.velocity * sin(self.initial_trim_case.fuselage_tilt)
        return StabilityDerivatives(u=u, w=w, q=0, theta_f=self.initial_trim_case.fuselage_tilt,
                                    collective_pitch=self.initial_trim_case.collective_pitch,
                                    longitudinal_cyclic=self.initial_trim_case.longitudinal_cyclic)

    @Attribute
    def u_derivatives(self):
        return np.array(self.stability_derivatives.u_derivatives)

    @Attribute
    def w_derivatives(self):
        return np.array(self.stability_derivatives.w_derivatives)

    @Attribute
    def q_derivatives(self):
        return np.array(self.stability_derivatives.q_derivatives)

    @Attribute
    def theta_f_derivatives(self):
        return np.array(self.stability_derivatives.theta_f_derivatives)

    @Attribute
    def collective_derivatives(self):
        return np.array(self.stability_derivatives.collective_derivatives)

    @Attribute
    def cyclic_derivatives(self):
        return np.array(self.stability_derivatives.cyclic_derivatives)

    @Attribute
    def a_matrix(self):
        column_1 = self.u_derivatives
        column_2 = self.w_derivatives
        column_3 = self.q_derivatives
        column_4 = self.theta_f_derivatives
        A = np.matrix([column_1, column_2, column_3, column_4]).T
        return A

    @Attribute
    def b_matrix(self):
        column_1= self.collective_derivatives
        column_2 = self.cyclic_derivatives
        B = np.matrix([column_1, column_2]).T
        return B

    @Attribute
    def state_space_solver(self):
        I = np.array([1, 1, 1, 1])
        C = np.diag(I)
        D = np.zeros((4, 2))
        # return C
        return ss(self.a_matrix, self.b_matrix, C, D)

    def step_response(self):
        sys2 = self.state_space_solver

        T = np.linspace(0, 10, 1000)
        cyclic_input = [0]          # delta!!!
        collective_input = [0]

        for i in range(1, len(T)):
            if 0.5 < T[i] < 1.0:
                cyclic_input.append(radians(20.0))
            else:
                cyclic_input.append(cyclic_input[0])
            collective_input.append(collective_input[0])

        print cyclic_input
        print collective_input

        U = np.array([collective_input, cyclic_input]).T
        #U = np.zeros([len(T), 2])

        yout, T, xout = lsim(sys2, U=U, T=T, input=1)

        print xout

        plt.figure(1)
        plt.plot(T, [num+self.stability_derivatives.u for num in yout[:, 0]], ':', linewidth=2, label='u')  # U IS CORRECT!
        plt.plot(T, [num+self.stability_derivatives.w for num in yout[:, 1]], ':', linewidth=2, label='w')
        plt.plot(T, [degrees(rad + self.stability_derivatives.q) for rad in yout[:, 2]], ':', linewidth=2, label='q')  #  q IS CORRECT
        plt.plot(T, [degrees(rad + self.stability_derivatives.theta_f) for rad in yout[:, 3]], ':', linewidth=2, label='theta_f')  #  theta_f IS CORRECT
        # print t2.shape, np.array(y2)
        plt.xlabel('Time')
        plt.ylabel('Response (y)')
        plt.legend(loc='best')
        plt.show()
        return 'Plotted'


if __name__ == '__main__':
    obj = StateSpace(1.0)
    print 'State Variables = u, w, q, theta_f'
    print 'u Derivatives ', obj.u_derivatives
    print 'w Derivatives ', obj.w_derivatives
    print 'q Derivatives ', obj.q_derivatives
    print 'theta_f Derivatives ', obj.theta_f_derivatives
    print 'collective Derivatives ', obj.collective_derivatives
    print 'cyclic Derivatives ', obj.cyclic_derivatives
    print 'A Matrix', obj.a_matrix
    print 'B Matrix', obj.b_matrix
    # print obj.state_space_solver
    obj.step_response()

