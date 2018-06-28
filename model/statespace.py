#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

# http://python-control.readthedocs.io/en/latest/intro.html most likely this will be required

__author__ = ["San Kilkis", "Nelson Johnson"]

from globs import Constants, Attribute
from stabilityderivatives import StabilityDerivatives
from trim import Trim
from control.matlab import *
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

    # TODO Consider moving u and w into the class Trim
    @Attribute
    def stability_derivatives(self):
        return StabilityDerivatives(u=self.initial_trim_case.u, w=self.initial_trim_case.w, q=0,
                                    theta_f=self.initial_trim_case.fuselage_tilt,
                                    collective_pitch=self.initial_trim_case.collective_pitch,
                                    longitudinal_cyclic=self.initial_trim_case.longitudinal_cyclic)

    @Attribute
    def a_matrix(self):
        column_1 = np.array(self.stability_derivatives.u_derivatives)
        column_2 = np.array(self.stability_derivatives.w_derivatives)
        column_3 = np.array(self.stability_derivatives.q_derivatives)
        column_4 = np.array(self.stability_derivatives.theta_f_derivatives)

        base_matrix = np.matrix([column_1, column_2, column_3, column_4]).T

        # added_row = np.zeros((5, 4))
        # added_row[:-1, :] = base_matrix
        #
        # added_column = np.zeros((5, 5))
        # added_column[:, :-1] = added_row

        # added_column[4, 3] = -1.0  # Fuselage error depends on the current fuselage angle theta_f
        return base_matrix

    @Attribute
    def b_matrix(self):
        column_1 = np.array(self.stability_derivatives.collective_derivatives)
        column_2 = np.array(self.stability_derivatives.cyclic_derivatives)

        base_matrix = np.matrix([column_1, column_2]).T

        # added_row = np.zeros((5, 2))
        # added_row[:-1, :] = base_matrix
        #
        # added_column = np.zeros((5, 3))
        # added_column[:, :-1] = added_row

        # added_column[4, 2] = 1.0  # Fuselage error depends on the current fuselage angle theta_f
        return base_matrix

    @Attribute
    def c_matrix(self):
        return np.diag([1, 1, 1, 1])

    @Attribute
    def d_matrix(self):
        return np.zeros((4, 2))

    @Attribute
    def system(self):
        # C = np.diag([1, 1, 1, 1, 0])
        # C[4, 3] = -1.0  # Fuselage error depends on the current fuselage angle theta_f
        # D = np.zeros((5, 3))
        # D[4, 2] = 1
        C = np.diag([1, 1, 1, 1])
        D = np.zeros((4, 2))
        return ss(self.a_matrix, self.b_matrix, self.c_matrix, self.d_matrix)

    # @Attribute
    # def controller(self):
    #     a_matrix = np.matrix([[2.0, 0, 0], [0, 0, 0], [0, 0, 0]])  # Integrated Control
    #     b_matrix = np.zeros((3, 5))
    #     c_matrix = np.diag([1, 1, 1])
    #     d_matrix = np.matrix([[0, 0, 0, 0, 0], [0, 0, 0.2, 0, -0.2], [0, 0, 0, 0, 0]])  # Proportional Control
    #     return ss(a_matrix, b_matrix, c_matrix, d_matrix)

    def step_response(self):
        sys1 = self.system
        # sys1 = feedback(sys1, self.controller, +1)

        T = np.linspace(0, 40, 1000)
        cyclic_input = [0]          # delta!!!
        collective_input = [0]
        desired_pitch = [-self.stability_derivatives.theta_f]

        for i in range(1, len(T)):
            if 0.5 < T[i] < 1.0:
                cyclic_input.append(radians(0.0))
            else:
                cyclic_input.append(cyclic_input[0])
            collective_input.append(collective_input[0])
            desired_pitch.append(-self.stability_derivatives.theta_f)

        U = np.array([collective_input, cyclic_input]).T

        yout, T, xout = lsim(sys1, U=U, T=T)

        plt.figure(1)
        plt.style.use('ggplot')
        plt.plot(T, [num+self.stability_derivatives.u for num in yout[:, 0]], label='u')  # U IS CORRECT!
        plt.plot(T, [num+self.stability_derivatives.w for num in yout[:, 1]], label='w')
        # plt.plot(T, [degrees(rad + self.stability_derivatives.q) for rad in yout[:, 2]], label='q')  #  q IS CORRECT
        # plt.plot(T, [degrees(rad + self.stability_derivatives.theta_f) for rad in yout[:, 3]], label='theta_f')  #  theta_f IS CORRECT
        # print t2.shape, np.array(y2)
        plt.xlabel('Time')
        plt.ylabel('Response (y)')
        plt.legend(loc='best')
        plt.show()

        plt.figure(2)
        plt.style.use('ggplot')
        plt.plot(T, [degrees(num+self.stability_derivatives.q) for num in yout[:, 2]], label='q')  # U IS CORRECT!
        plt.plot(T, [degrees(num+self.stability_derivatives.theta_f) for num in yout[:, 3]], label='theta_f')
        # plt.plot(T, [degrees(rad + self.stability_derivatives.q) for rad in yout[:, 2]], label='q')  #  q IS CORRECT
        # plt.plot(T, [degrees(rad + self.stability_derivatives.theta_f) for rad in yout[:, 3]], label='theta_f')  #  theta_f IS CORRECT
        # print t2.shape, np.array(y2)
        plt.xlabel('Time')
        plt.ylabel('Response (y)')
        plt.legend(loc='best')
        plt.show()

        # plt.figure(3)
        # plt.style.use('ggplot')
        # plt.plot(T, [degrees(num) for num in yout[:, 4]], label='theta_f')
        # # plt.plot(T, [degrees(rad + self.stability_derivatives.q) for rad in yout[:, 2]], label='q')  #  q IS CORRECT
        # # plt.plot(T, [degrees(rad + self.stability_derivatives.theta_f) for rad in yout[:, 3]], label='theta_f')  #  theta_f IS CORRECT
        # # print t2.shape, np.array(y2)
        # plt.xlabel('Time')
        # plt.ylabel('Response (y)')
        # plt.legend(loc='best')
        # plt.show()
        return 'Plotted'


    # def step_response(self):
    #     sys2 = self.state_space_solver
    #
    #     T = np.linspace(0, 10, 1000)
    #     cyclic_input = [0]          # delta!!!
    #     collective_input = [0]
    #
    #     for i in range(1, len(T)):
    #         if 0.5 < T[i] < 1.0:
    #             cyclic_input.append(radians(1.0))
    #         else:
    #             cyclic_input.append(cyclic_input[0])
    #         collective_input.append(collective_input[0])
    #
    #     print cyclic_input
    #     print collective_input
    #
    #     U = np.array([collective_input, cyclic_input]).T
    #
    #     yout, T, xout = lsim(sys2, U=U, T=T)
    #
    #     print xout
    #
    #     time = T
    #     delta_t = time[1] - time[0]
    #     cyclic_input = [0]
    #     u = [self.stability_derivatives.u]
    #     w = [self.stability_derivatives.w]
    #     q = [self.stability_derivatives.q]
    #     theta_f = [self.stability_derivatives.theta_f]
    #     current_case = self.stability_derivatives
    #
    #     # Forward Euler Integration
    #     for i in range(1, len(time)):
    #         u.append(current_case.u + current_case.u_dot * delta_t)
    #         w.append(current_case.w + current_case.w_dot * delta_t)
    #         q.append(current_case.q + current_case.q_dot * delta_t)
    #         theta_f.append(current_case.theta_f + current_case.theta_f_dot * delta_t)
    #
    #         # Control Inputs based on Initial Conditions
    #         if 0.5 < time[i] < 1.0:
    #             cyclic_input.append(self.stability_derivatives.longitudinal_cyclic + radians(1.0))
    #         else:
    #             cyclic_input.append(self.stability_derivatives.longitudinal_cyclic)
    #
    #         current_case = StabilityDerivatives(u=u[i], w=w[i], q=q[i], theta_f=theta_f[i],
    #                                             longitudinal_cyclic=cyclic_input[i],
    #                                             collective_pitch=self.stability_derivatives.collective_pitch)
    #
    #
    #     plt.figure(1)
    #     plt.style.use('ggplot')
    #     plt.plot(T, [num+self.stability_derivatives.u for num in yout[:, 0]], label='u')  # U IS CORRECT!
    #     plt.plot(T, u, label='u Euler')
    #     plt.plot(T, [num+self.stability_derivatives.w for num in yout[:, 1]], label='w')
    #     plt.plot(T, w, label='w Euler')
    #     # plt.plot(T, [degrees(rad + self.stability_derivatives.q) for rad in yout[:, 2]], label='q')  #  q IS CORRECT
    #     # plt.plot(T, [degrees(rad + self.stability_derivatives.theta_f) for rad in yout[:, 3]], label='theta_f')  #  theta_f IS CORRECT
    #     # print t2.shape, np.array(y2)
    #     plt.xlabel('Time')
    #     plt.ylabel('Response (y)')
    #     plt.legend(loc='best')
    #     plt.show()
    #     return 'Plotted'


if __name__ == '__main__':
    obj = StateSpace(5.14444)
    # print 'State Variables = u, w, q, theta_f'
    # print 'u Derivatives ', obj.u_derivatives
    # print 'w Derivatives ', obj.w_derivatives
    # print 'q Derivatives ', obj.q_derivatives
    # print 'theta_f Derivatives ', obj.theta_f_derivatives
    # print 'collective Derivatives ', obj.collective_derivatives
    # print 'cyclic Derivatives ', obj.cyclic_derivatives
    print 'A Matrix', obj.a_matrix
    print 'B Matrix', obj.b_matrix
    print 'Fuselage Pitch', degrees(obj.stability_derivatives.theta_f)

    # print obj.system
    obj.step_response()

