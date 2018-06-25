#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

__author__ = ["San Kilkis"]

from globs import Constants, Attribute

import numpy as np
from numpy.linalg import inv
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from math import sqrt, pi, degrees, cos, sin, atan
import os  # Necessary to determining the current working directory to save figures

_working_dir = os.getcwd()


class Trim(Constants):
    """ Computes the trim condition of the CH-53 based on input of current forward flight velocity

    :param velocity: Forward Flight Velocity in SI meter per second [m/s]
    :type velocity: float
    """

    def __init__(self, velocity=0.0):
        self.velocity = float(velocity)

    @Attribute
    def drag(self):
        """ Computes the Drag Force acting on the fuselage utilizing the Equivalent Flat Plate Area

        :return: Drag Force in SI Newton [N]
        :rtype: float
        """
        return self.flat_plate_area*0.5*self.rho*(self.velocity**2)

    @Attribute
    def thrust(self):
        """ Computes the Thrust Force based on the assumption that during trim the thust is equal in magnitude and
        opposite in direction to the resultant force due to the the drag and weight force vectors

        :return: Thrust Force in SI Newton [N]
        :rtype: float
        """
        return sqrt(self.drag**2 + self.weight_mtow**2)

    @Attribute
    def thrust_coef(self):
        """ Computes the Non-Dimensional Thrust Coefficient as per S.7 of Lecture 11 Part I

        :return: Non-Dimensionalized Thrust Coefficient
        :return: float
        """
        return self.thrust / (self.rho * ((self.main_rotor.omega * self.main_rotor.radius)**2) * pi
                              * (self.main_rotor.radius**2))

    @Attribute
    def alpha_disk(self):
        """ Computes the Disk Angle of Attack (AoA) in the Tip Path Plane (TPP)

        :return: Angle of Attack in the Tip Path Plane in SI radian [rad]
        :rtype: float
        """
        return atan(self.drag/self.weight_mtow)

    @Attribute
    def hover_induced_velocity(self):
        """ Utilizes the ACT Definition from Assignment I to calculate the hover induced velocity

        :return: Hover Induced Velocity in SI meter per second [m/s]
        """
        return sqrt(self.weight_mtow/(2*self.rho*pi*(self.main_rotor.radius**2)))

    @Attribute
    def normalized_velocity(self):
        return self.velocity / self.hover_induced_velocity

    @Attribute
    def induced_velocity(self):

        def func(x, *args):
            """ Defines a 2-th order equation function (V*sin(a) + v_i)**2 + (V*cos(a))**2 - (1/v_i)**2 = 0

            :param x: Represents the Non-Dimensional Induced Velocity
            """
            v_bar, alpha_disk = args
            return ((v_bar * sin(alpha_disk) + abs(x)) ** 2 + (v_bar * cos(alpha_disk)) ** 2) - (
                    1 / (abs(x) ** 2))

        return abs(fsolve(func, x0=np.array([1]), args=(self.alpha_disk,
                                                        self.normalized_velocity))[0]) * self.hover_induced_velocity

    @Attribute
    def inflow_ratio(self):
        """ Computes the Inflow-Ratio utilizing the """
        return self.induced_velocity / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def inflow_ratio_glau(self):
        """ Utilizes a numerical solver with the Glauert Theory Thrust Coefficient equation to find the Inflow Ratio
        at the current velocity

        :return: Inflow Ratio
        :return: float
        """

        def func(lambda_i, *args):
            v, omega, r, alpha_d, ct = args
            return 2 * lambda_i * sqrt(((v / (omega * r)) * cos(alpha_d)) ** 2 + ((v / (omega * r)) * sin(alpha_d) +
                                                                                  lambda_i) ** 2) - ct

        return fsolve(func, x0=np.array([1.]), args=(self.velocity,
                                                     self.main_rotor.omega,
                                                     self.main_rotor.radius,
                                                     self.alpha_disk,
                                                     self.thrust_coef))[0]

    @Attribute
    def numerical_solution(self):
        """ This attribute computes the trim solution at the current forward flight velocity

        :returns: Collective Pitch and Longitudinal Cyclic in SI radian [rad]
        :rtype: tuple
        """

        def func(variables, *args):
            """ Numerically solves the system of 2 equations with 2 unknowns """

            # Retrieving the Collective Pitch and Longitudinal
            theta0, thetac = variables

            # Retrieving the Passed Arguments
            velocity, alpha_disk, main_rotor, cla, lambda_i, ct = args

            alpha_control = alpha_disk + thetac  # Computing the Disk AoA at the Control Plane CP
            mu = (velocity * cos(alpha_control)) / (main_rotor.omega * main_rotor.radius)  # Advance Ratio
            lambda_c = velocity * sin(alpha_control) / (main_rotor.omega * main_rotor.radius)  # Inflow Ratio at CP

            def thrust_constraint(collective_pitch, advance_ratio, inflow_ratio_control, inflow_ratio, lift_gradient,
                                  thrust_coef, solidity):
                """ Relation depicting the thrust from Blade Element Momentum Theory utilized to constrain the trim
                solution

                :return: Difference between the BEM Thrust Coefficient and the Required Thrust Coefficient
                :rtype: float
                """
                return (lift_gradient * solidity / 4.) * \
                       (((2. / 3.) * collective_pitch) * (1 + (3. / 2.) * advance_ratio ** 2) - (
                               inflow_ratio + inflow_ratio_control)) - thrust_coef

            def cyclic_constraint(collective_pitch, longitudinal_cyclic, advance_ratio, inflow_ratio_control,
                                  inflow_ratio):
                """ Relation depicting the longitudinal cyclic w/ the assumption that the Tip Path Plane (TPP)
                coincides w/ the Shaft Plane (SP). This is utilized to constrain the trim solution

                :return: Difference between the Longitudinal Cyclic and the Blade Tilt Angle in SI radian [rad]
                """
                return (((8. / 3.) * advance_ratio * collective_pitch - 2. * advance_ratio * (inflow_ratio +
                                                                                              inflow_ratio_control)) /
                        (1 - 0.5 * advance_ratio ** 2)) - longitudinal_cyclic

            thrust_output = thrust_constraint(theta0, mu, lambda_c, lambda_i, cla, ct, main_rotor.solidity)
            cyclic_output = cyclic_constraint(theta0, thetac, mu, lambda_c, lambda_i)

            return thrust_output, cyclic_output

        sol = fsolve(func, x0=np.array([0., 0.]), args=(self.velocity,
                                                        self.alpha_disk,
                                                        self.main_rotor,
                                                        self.lift_gradient,
                                                        self.inflow_ratio_glau,
                                                        self.thrust_coef))

        return sol[0], sol[1]

    @Attribute
    def linearized_solution(self):

        # Linearizing the Advance-Ratio by neglecting the effect of the Longitudinal Cyclic (Small Angles)
        mu = self.velocity / (self.main_rotor.omega * self.main_rotor.radius)

        # Creating the A-matrix of the system
        a_matrix = np.array([[1 + (3./2.) * mu ** 2, (-8./3.) * mu], [-mu,  2./3. + mu**2]])

        # Creating the b-matrix of the system
        b1 = -2. * mu**2 * self.alpha_disk - 2 * mu * self.inflow_ratio_glau
        b2 = (4. * self.thrust_coef)/(self.main_rotor.solidity * self.lift_gradient) + mu * self.alpha_disk + \
            self.inflow_ratio_glau
        b = np.array([[b1], [b2]])

        # Obtaining solution by pre-multiplying the inverse of the A-matrix with the b-matrix
        sol = np.matmul(inv(a_matrix), b)

        return sol[1], sol[0]

    @staticmethod
    def plot_error():
        velocities = np.linspace(0.1, 100, 50)
        trim_conditions = [Trim(v) for v in velocities]
        errors = [((case.inflow_ratio_glau - case.inflow_ratio) / case.inflow_ratio) * 100. for case in trim_conditions]
        fig = plt.figure('ErrorvsVelocity')
        plt.style.use('ggplot')
        plt.plot(velocities, errors)
        plt.title(r'Inflow Ratio Error as a Function of Velocity')
        plt.xlabel(r'True Airspeed, $V_\mathrm{TAS}$ [m/s]')
        plt.ylabel(r'Percentage Error [%]')
        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
        return '%s Plotted and Saved' % fig.get_label()

    def plot_trim(self):
        velocities = np.linspace(0, self.cruise_velocity + 10, 100)
        trim_conditions = [Trim(v) for v in velocities]

        # Retrieving Numerical Solution
        pitch_num = [degrees(case.numerical_solution[0]) for case in trim_conditions]
        cyclic_num = [degrees(case.numerical_solution[1]) for case in trim_conditions]

        # Retrieving Linearized Solution
        pitch_lin = [degrees(case.linearized_solution[0]) for case in trim_conditions]
        cyclic_lin = [degrees(case.linearized_solution[1]) for case in trim_conditions]

        # Plotting Numerical Solution
        fig = plt.figure('TrimvsVelocity')
        plt.style.use('ggplot')
        plt.plot(velocities, pitch_num, label='Collective')
        plt.plot(velocities, cyclic_num, label='Cyclic')

        # Plotting Linearized Solution
        plt.plot(velocities, pitch_lin, linewidth=1.0, linestyle=':', color='black', label='Linearized Solution')
        plt.plot(velocities, cyclic_lin, linewidth=1.0, linestyle=':', color='black')

        # Creating Labels & Saving Figure
        plt.title(r'Control Deflection as a Function of Forward Velocity')
        plt.xlabel(r'True Airspeed, $V_\mathrm{TAS}$ [m/s]')
        plt.ylabel(r'Required Control Deflection [deg]')
        plt.legend(loc='best')
        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
        return '%s Plotted and Saved' % fig.get_label()


if __name__ == '__main__':
    obj = Trim()
    obj.plot_trim()
    obj.plot_error()
