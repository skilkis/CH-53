#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the Linearized EoM of the CH53 Helicopter """

__author__ = ["San Kilkis", "Nelson Johnson"]

import __root__
from globs import Constants, Attribute, working_dir
from stabilityderivatives import StabilityDerivatives
from trim import Trim
from control.matlab import ss, lsim, np
from math import radians, degrees
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import os
from utils import ProgressBar
assert __root__  # Necessary to circumvent PEP-8 Syntax violation on the __root__ import statement


class StateSpace(Constants):
    """ Obtains the stability derivatives at a certain trim condition calculated from :parameter:`initial_velocity`
    utilizing the :class:`StabilityDerivatives` and organizes them into state-space system. A sample response is also
    provided for a Phugoid excitation to be able to compare results to the non-linear equations of motion.

    :param initial_velocity: Initial velocity of the CH-53 in SI meter per second [m/s]
    :type initial_velocity: float
    """

    def __init__(self, initial_velocity=0.0):
        self.initial_velocity = initial_velocity

    @Attribute
    def initial_trim_case(self):
        """ Passes the :parameter:`initial_velocity` to the :class:`Trim` to obtain state variables at trim condition

        :return: Trim object containing all state-variables at the desired trim condition
        :rtype: Trim
        """
        return Trim(self.initial_velocity)

    @Attribute
    def stability_derivatives(self):
        prog = ProgressBar('Instantiating Stability Object')
        derivatives = StabilityDerivatives(u=self.initial_trim_case.u, w=self.initial_trim_case.w, q=0,
                                           theta_f=self.initial_trim_case.fuselage_tilt,
                                           collective_pitch=self.initial_trim_case.collective_pitch,
                                           longitudinal_cyclic=self.initial_trim_case.longitudinal_cyclic)
        prog.update(100)
        return derivatives

    @Attribute
    def a_matrix(self):
        """ Retrieves the relevant stability derivatives and organizes them into the state or system matrix A

        :return: A-matrix
        :rtype: np.matrix
        """
        column_1 = np.array(self.stability_derivatives.u_derivatives)
        column_2 = np.array(self.stability_derivatives.w_derivatives)
        column_3 = np.array(self.stability_derivatives.q_derivatives)
        column_4 = np.array(self.stability_derivatives.theta_f_derivatives)
        return np.matrix([column_1, column_2, column_3, column_4]).T

    @Attribute
    def b_matrix(self):
        """ Retrieves the relevant stability derivatives and organizes them into the input matrix B

        :return: B-matrix
        :rtype: np.matrix
        """
        column_1 = np.array(self.stability_derivatives.collective_derivatives)
        column_2 = np.array(self.stability_derivatives.cyclic_derivatives)
        return np.matrix([column_1, column_2]).T

    @Attribute
    def c_matrix(self):
        """ Returns a 4x4 identity matrix since simply a new state-variable is desired

        :return: 4x4 Identity C-matrix
        :rtype: np.diag
        """
        return np.diag([1., 1., 1., 1.])

    @Attribute
    def d_matrix(self):
        """ The system has no feed-forward for the output vector thus a 4x2 matrix of zeros are required

        :return: 4x2 Matrix of zeroes for the Feed-forward D-matrix
        :rtype: np.zeros
        """
        return np.zeros((4, 2))

    @Attribute
    def system(self):
        """ Organizes the obtained state-space matrices into a state-space object utilizing :class:`ss`

        :return: State-space system utilizing the control library
        :rtype: control.matlab.ss
        """
        return ss(self.a_matrix, self.b_matrix, self.c_matrix, self.d_matrix)

    @staticmethod
    def plot_derivatives():
        # TODO Finish plotting these derivatives if necessary for the report
        pbar = ProgressBar('Obtaining Stability Derivatives for Entire Flight Envelope')
        velocities = np.linspace(0, 75, 20)
        x_u = []
        x_w = []

        for i, v in enumerate(velocities):
            trim_case = Trim(v)
            derivatives = StabilityDerivatives(u=trim_case.u, w=trim_case.w, q=0,
                                               theta_f=trim_case.fuselage_tilt,
                                               collective_pitch=trim_case.collective_pitch,
                                               longitudinal_cyclic=trim_case.longitudinal_cyclic)
            x_u.append(derivatives.u_derivatives[0])
            x_w.append(derivatives.u_derivatives[1])
            pbar.update_loop(i, len(velocities)-1, 'V = %1.2f [m/s]' % v)

        plt.plot(velocities, x_u)
        plt.plot(velocities, x_w)
        plt.show()

        return 'Figure Plotted and Saved'

    def plot_response(self):
        """ Plots the response of the CH-53 to a step input of 1 [deg] longitudinal cyclic. Note that the state-space
        representation models the change in inputs and state-variables, thus to use the same input vectors the initial
        condition is set to zero control deflection. This is then compensated by adding the control deflection
        necessary for trim during the Forward Euler integration for the non-linear equations. """

        pbar = ProgressBar('Performing Linear Simulation')
        time = np.linspace(0, 40, 1000)

        # Initial Input Conditions
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
        yout, time, xout = lsim(self.system, U=input_matrix, T=time)

        pbar.update(100)

        delta_t = time[1] - time[0]
        u = [self.stability_derivatives.u]
        w = [self.stability_derivatives.w]
        q = [self.stability_derivatives.q]
        theta_f = [self.stability_derivatives.theta_f]
        current_case = self.stability_derivatives

        # Forward Euler Integration
        pbar = ProgressBar('Performing Forward Euler Integration')
        for i in range(1, len(time)):
            u.append(current_case.u + current_case.u_dot * delta_t)
            w.append(current_case.w + current_case.w_dot * delta_t)
            q.append(current_case.q + current_case.q_dot * delta_t)
            theta_f.append(current_case.theta_f + current_case.theta_f_dot * delta_t)

            current_case = StabilityDerivatives(u=u[i], w=w[i], q=q[i], theta_f=theta_f[i],
                                                longitudinal_cyclic=cyclic_input[i] +
                                                                    self.initial_trim_case.longitudinal_cyclic,
                                                collective_pitch=self.initial_trim_case.collective_pitch)
            pbar.update_loop(i, len(time)-1)

        # Creating First Figure
        plt.style.use('ggplot')
        fig, (cyc_plot, vel_plot) = plt.subplots(2, 1, num='EulervsStateVelocities', sharex='all')

        # Plotting Input
        cyc_plot.plot(time, [degrees(rad) for rad in cyclic_input])
        cyc_plot.set_ylabel(r'Lon. Cyclic [deg]')
        cyc_plot.set_xlabel('')
        cyc_plot.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
        cyc_plot.set_title(r'Velocity Response as a Function of Time')

        # Plotting Velocity Response
        vel_plot.plot(time, u, label='Horizontal')
        vel_plot.plot(time, w, label='Vertical')
        vel_plot.plot(time, [num+self.stability_derivatives.u for num in yout[:, 0]], linestyle=':', color='black')
        vel_plot.plot(time, [num+self.stability_derivatives.w for num in yout[:, 1]], linestyle=':', color='black',
                      label='Lin. Simulation')
        vel_plot.set_ylabel(r'Velocity [m/s]')
        vel_plot.set_xlabel('')
        vel_plot.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        vel_plot.legend(loc='best')

        # Creating Labels & Saving Figure
        plt.xlabel(r'Time [s]')
        plt.show()
        fig.savefig(fname=os.path.join(working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

        # Creating Second Figure
        fig, (cyc_plot, q_plot, theta_plot) = plt.subplots(3, 1, num='EulervsStateAngles', sharex='all')
        cyc_plot.plot(time, [degrees(rad) for rad in cyclic_input])

        # Plotting Input
        cyc_plot.set_ylabel(r'$\theta_{ls}$ [deg]')
        cyc_plot.yaxis.set_major_formatter(FormatStrFormatter('%.3f'))
        cyc_plot.set_title(r'Angular Response as a Function of Time')

        # Plotting Fuselage Pitch-Rate
        q_plot.plot(time, [degrees(rad) for rad in q])
        q_plot.plot(time, [degrees(num+self.stability_derivatives.q) for num in yout[:, 2]],
                    linestyle=':',
                    color='black',
                    label='Lin. Simulation')
        q_plot.set_ylabel(r'$q$ [deg/s]')
        q_plot.set_xlabel('')
        q_plot.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        q_plot.legend(loc='best')

        # Plotting Fuselage Pitch
        theta_plot.plot(time, [degrees(rad) for rad in theta_f])
        theta_plot.plot(time, [degrees(num+self.stability_derivatives.theta_f) for num in yout[:, 3]],
                        linestyle=':',
                        color='black',
                        label='Lin. Simulation')
        theta_plot.set_ylabel(r'$\theta_f$ [deg]')
        theta_plot.set_xlabel('')
        theta_plot.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
        theta_plot.legend(loc='best')

        # Creating x-label & Saving Figure
        plt.xlabel(r'Time [s]')
        plt.show()
        fig.savefig(fname=os.path.join(working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

        return 'Figures Plotted and Saved'


if __name__ == '__main__':
    obj = StateSpace(initial_velocity=5.1444)
    print ('A Matrix\n%s\n' % obj.a_matrix)
    print ('B Matrix\n%s\n' % obj.b_matrix)
    print ('Fuselage Pitch %s [deg]' % degrees(obj.stability_derivatives.theta_f))
    # obj.plot_derivatives()
    obj.plot_response()

