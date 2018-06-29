#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

# http://python-control.readthedocs.io/en/latest/intro.html most likely this will be required

__author__ = ["San Kilkis", "Nelson Johnson"]

import __root__
from globs import Constants, Attribute, working_dir
from stabilityderivatives import StabilityDerivatives
from trim import Trim
from control.matlab import *
from math import radians, degrees
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import FormatStrFormatter
import os
from timeit import default_timer as timer
assert __root__


class StateSpace(Constants):
    """ xxx

    :param u: xx
    :type u: xx
    """

    def __init__(self, initial_velocity=0.0):
        self.initial_velocity = initial_velocity

    @Attribute
    def initial_trim_case(self):
        return Trim(self.initial_velocity)

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
        return np.matrix([column_1, column_2, column_3, column_4]).T

    @Attribute
    def b_matrix(self):
        column_1 = np.array(self.stability_derivatives.collective_derivatives)
        column_2 = np.array(self.stability_derivatives.cyclic_derivatives)
        return np.matrix([column_1, column_2]).T

    @Attribute
    def c_matrix(self):
        return np.diag([1., 1., 1., 1.])

    @Attribute
    def d_matrix(self):
        return np.zeros((4, 2))

    @Attribute
    def system(self):
        return ss(self.a_matrix, self.b_matrix, self.c_matrix, self.d_matrix)


    def plot_derivatives(self):
        velocities = np.linspace(0, 75, 20)
        x_u = []
        x_w = []

        for v in velocities:
            trim_case = Trim(v)
            derivatives = StabilityDerivatives(u=trim_case.u, w=trim_case.w, q=0,
                                               theta_f=trim_case.fuselage_tilt,
                                               collective_pitch=trim_case.collective_pitch,
                                               longitudinal_cyclic=trim_case.longitudinal_cyclic)
            x_u.append(derivatives.u_derivatives[0])
            x_w.append(derivatives.u_derivatives[1])

        plt.plot(velocities, x_u)
        plt.plot(velocities, x_w)
        plt.show()

        return 'Figure Plotted and Saved'

    def step_response(self):
        sys1 = self.system
        # sys1 = feedback(sys1, self.controller, +1)

        T = np.linspace(0, 40, 1000)
        cyclic_input = [0]          # delta!!!
        collective_input = [0]

        # Defining Step Input into the Longitudinal Cyclic
        for i in range(1, len(T)):
            if 0.5 < T[i] < 1.0:
                cyclic_input.append(radians(0.0))
            else:
                cyclic_input.append(cyclic_input[0])
            collective_input.append(collective_input[0])

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
        return 'Plotted'

    def plot_comparison(self):
        start = timer()
        linearized_system = self.system

        time = np.linspace(0, 40, 1000)

        # Initial Conditions
        cyclic_input = [0]
        collective_input = [0]

        # Defining Step Input into the Longitudinal Cyclic
        for i in range(1, len(time)):
            if 0.5 < time[i] < 1.0:
                cyclic_input.append(radians(1.0))
            else:
                cyclic_input.append(cyclic_input[0])
            collective_input.append(collective_input[0])

        # Input Matrix U for the State-Space Simulation
        input_matrix = np.array([collective_input, cyclic_input]).T

        # Simulating the Linear System
        yout, time, xout = lsim(linearized_system, U=input_matrix, T=time)
        end = timer()

        print ('\nLinear Simulation Performed \n' + 'Duration: %1.5f [s]\n' % (end - start))

        delta_t = time[1] - time[0]
        u = [self.stability_derivatives.u]
        w = [self.stability_derivatives.w]
        q = [self.stability_derivatives.q]
        theta_f = [self.stability_derivatives.theta_f]
        current_case = self.stability_derivatives

        # Forward Euler Integration
        start = timer()
        for i in range(1, len(time)):
            u.append(current_case.u + current_case.u_dot * delta_t)
            w.append(current_case.w + current_case.w_dot * delta_t)
            q.append(current_case.q + current_case.q_dot * delta_t)
            theta_f.append(current_case.theta_f + current_case.theta_f_dot * delta_t)

            current_case = StabilityDerivatives(u=u[i], w=w[i], q=q[i], theta_f=theta_f[i],
                                                longitudinal_cyclic=cyclic_input[i] +
                                                                    self.initial_trim_case.longitudinal_cyclic,
                                                collective_pitch=self.initial_trim_case.collective_pitch)

        end = timer()
        print ('Integration Performed \n' + 'Duration: %1.5f [s]\n' % (end - start))

        # Plotting Response
        fig = plt.figure('EulervsStateVelocities')
        plt.style.use('ggplot')
        gs = gridspec.GridSpec(2, 1, top=0.9)

        cyc_plot = fig.add_subplot(gs[0, 0])
        cyc_plot.plot(time, [degrees(rad) for rad in cyclic_input])
        cyc_plot.set_ylabel(r'Lon. Cyclic [deg]')
        cyc_plot.set_xlabel('')
        cyc_plot.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))

        vel_plot = fig.add_subplot(gs[1, 0])
        vel_plot.plot(time, u, label='Horizontal')
        vel_plot.plot(time, w, label='Vertical')
        plt.plot(time, [num+self.stability_derivatives.u for num in yout[:, 0]], linestyle=':', color='black')
        plt.plot(time, [num+self.stability_derivatives.w for num in yout[:, 1]], linestyle=':', color='black',
                 label='Lin. Simulation')
        vel_plot.set_ylabel(r'Velocity [m/s]')
        vel_plot.set_xlabel('')
        vel_plot.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        vel_plot.legend(loc='best')

        # Creating Labels & Saving Figure
        plt.suptitle(r'Velocity Response as a Function of Time')
        plt.xlabel(r'Time [s]')
        plt.show()
        fig.set_tight_layout('False')
        fig.savefig(fname=os.path.join(working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

        # Creating Second Figure
        fig = plt.figure('EulervsStateAngles')
        plt.style.use('ggplot')
        gs = gridspec.GridSpec(3, 1, top=0.925, left=0.15)

        cyc_plot = fig.add_subplot(gs[0, 0])
        cyc_plot.plot(time, [degrees(rad) for rad in cyclic_input])
        cyc_plot.set_ylabel(r'$\theta_{ls}$ [deg]')
        cyc_plot.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))

        q_plot = fig.add_subplot(gs[1, 0])
        q_plot.plot(time, [degrees(rad) for rad in q])
        plt.plot(time, [degrees(num+self.stability_derivatives.q) for num in yout[:, 2]],
                 linestyle=':',
                 color='black',
                 label='Lin. Simulation')
        q_plot.set_ylabel(r'$q$ [deg/s]')
        q_plot.set_xlabel('')
        # q_plot.yaxis.set_major_formatter(FormatStrFormatter('%.1E'))
        q_plot.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
        q_plot.legend(loc='best')

        theta_plot = fig.add_subplot(gs[2, 0])
        theta_plot.plot(time, [degrees(rad) for rad in theta_f])
        plt.plot(time, [degrees(num+self.stability_derivatives.theta_f) for num in yout[:, 3]],
                 linestyle=':',
                 color='black',
                 label='Lin. Simulation')
        theta_plot.set_ylabel(r'$\theta_f$ [deg]')
        theta_plot.set_xlabel('')
        theta_plot.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
        theta_plot.legend(loc='best')

        # Creating Labels & Saving Figure
        plt.suptitle(r'Angular Response as a Function of Time')
        plt.xlabel(r'Time [s]')
        plt.show()
        fig.set_tight_layout('False')
        fig.savefig(fname=os.path.join(working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

        return 'Figures Plotted and Saved'


if __name__ == '__main__':
    obj = StateSpace(5.14444)
    print ('A Matrix', obj.a_matrix)
    print ('B Matrix', obj.b_matrix)
    print ('Fuselage Pitch', degrees(obj.stability_derivatives.theta_f))
    obj.plot_comparison()

