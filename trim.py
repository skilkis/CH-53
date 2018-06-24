#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

__author__ = ["San Kilkis"]

from globs import *

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
        return sqrt(self.weight_mtow/(2*self.rho*pi*(self.main_rotor.radius ** 2)))

    @Attribute
    def induced_velocity(self):
        v_bar = self.velocity / self.hover_induced_velocity

        def func(x):
            """ Defines a 2-th order equation function (V*sin(a) + v_i)**2 + (V*cos(a))**2 - (1/v_i)**2 = 0

            :param x: Represents the Non-Dimensional Induced Velocity
            """
            return ((v_bar * sin(self.alpha_disk) + abs(x)) ** 2 + (v_bar * cos(self.alpha_disk)) ** 2) - (
                    1 / (abs(x) ** 2))

        return abs(fsolve(func, x0=np.array([1]))[0]) * self.hover_induced_velocity

    @Attribute
    def inflow_ratio(self):
        """ Computes the Inflow-Ratio utilizing the """
        return self.induced_velocity / (self.main_rotor.omega * self.main_rotor.radius)

    def alpha_control(self, longitudinal_cyclic):
        """ Computes the Disk Angle of Attack (AoA) in the Control Plane (CP) based on the Longitudinal Cyclic Input

        :param longitudinal_cyclic: Longitudinal Cyclic in SI rad [rad]
        :type longitudinal_cyclic: float

        :return: Angle of Attack in the Control Plane in SI radian [rad]
        :rtype: float
        """
        return self.alpha_disk + longitudinal_cyclic

    def inflow_ratio_control(self, alpha_control):
        """ Computes the Non-Dimensional Inflow Ratio based on the current AoA of the Disk in the Control Plane (CP)

        :param alpha_control: Disk Angle of Attack (AoA) in the Control Plane in SI radian [rad]
        :type alpha_control: float

        :return: Inflow Ratio in the Control Plane
        :rtype: float
        """
        return self.velocity * sin(alpha_control) / (self.main_rotor.omega * self.main_rotor.radius)

    def advance_ratio(self, alpha_control):
        """ Computes the Advance Ratio, also referred to as the tip-speed ratio """
        return (self.velocity * cos(alpha_control)) / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def inflow_ratio_glau(self):
        """ Utilizes a numerical solver with the Glauert Theory Thrust Coefficient equation to find the Inflow Ratio
        at the current velocity

        :return: Inflow Ratio
        :return: float
        """

        def func(lambda_i, *args):
            v, omega, r, alpha_d, ct = args
            return 2 * abs(lambda_i) * sqrt(((v / (omega * r)) * cos(alpha_d)) ** 2 +
                                            ((v / (omega * r)) * sin(alpha_d) + abs(lambda_i)) ** 2) - ct

        return abs(fsolve(func, x0=np.array([0]), args=(self.velocity,
                                                        self.main_rotor.omega,
                                                        self.main_rotor.radius,
                                                        self.alpha_disk,
                                                        self.thrust_coef))[0])

    def thrust_constraint(self, collective_pitch, longitudinal_cyclic):
        """ Relation depicting the thrust from Blade Element Momentum Theory utilized to constrain the trim solution

        :return: Difference between the BEM Thrust Coefficient and the Required Thrust Coefficient
        :rtype: float
        """
        alpha_control = self.alpha_control(longitudinal_cyclic)
        mu = self.advance_ratio(alpha_control)
        lambda_c = self.inflow_ratio_control(longitudinal_cyclic)
        lambda_i = self.inflow_ratio
        return (self.lift_gradient * self.main_rotor.solidity / 4.) * \
               (((2./3.) * collective_pitch) * (1 + (3./2.) * mu**2) - (lambda_i + lambda_c)) - self.thrust_coef

    def cyclic_constraint(self, collective_pitch, longitudinal_cyclic):
        """ Relation depicting the longitudinal cyclic w/ the assumption that the Tip Path Plane (TPP) coincides w/
        the Shaft Plane (SP). This is utilized to constrain the trim solution

        :return: Difference between the Longitudinal Cyclic and the Blade Tilt Angle in SI radian [rad] """
        alpha_control = self.alpha_control(longitudinal_cyclic)
        mu = self.advance_ratio(alpha_control)
        lambda_c = self.inflow_ratio_control(longitudinal_cyclic)
        lambda_i = self.inflow_ratio
        return (((8./3.) * mu * collective_pitch - 2. * mu * (lambda_i + lambda_c)) /
                (1 - 0.5 * mu**2)) - longitudinal_cyclic

    @Attribute
    def trim_solver(self):
        """ This attribute computes the trim solution at the current forward flight velocity

        :returns: Collective Pitch and Longitudinal Cyclic in SI radian [rad]
        :rtype: tuple
        """

        def func(variables):
            """ Numerically solves the system of 2 equations with 2 unknowns """
            theta0, thetac = variables  # Un-packing Variables
            thrust_constraint = self.thrust_constraint(theta0, thetac)  # First Eq.
            cyclic_constraint = self.cyclic_constraint(theta0, thetac)  # Second Eq.
            return thrust_constraint, cyclic_constraint

        collective_pitch, longitudinal_cyclic = fsolve(func, x0=np.array([0., 0.]))

        return collective_pitch, longitudinal_cyclic

    @Attribute
    def test_solution(self):
        mu = self.velocity / (self.main_rotor.omega * self.main_rotor.radius)
        a1 = ((8./3.) * mu) / (1 - 0.5 * mu**2)
        b1 = (2 * mu) / (1 - 0.5 * mu**2)
        a2 = ((self.lift_gradient * self.main_rotor.solidity) / 4.) * (2./3.) * (1 + (3./2.) * mu**2)
        b2 = ((self.lift_gradient * self.main_rotor.solidity) / 4.)
        A = np.array([[a1, b1 * mu - 1], [a2,  b2 * mu]])
        b = np.array([[b1 * self.inflow_ratio - b1 * mu * self.alpha_disk],
                     [b2 * (self.inflow_ratio + mu * self.alpha_disk) + self.thrust_coef]])
        sol = np.matmul(inv(A), b)
        return sol

    @staticmethod
    def plot_error():
        velocities = np.linspace(0.1, 50, 50)
        trim_conditions = [Trim(v) for v in velocities]
        errors = [((case.inflow_ratio_glau - case.inflow_ratio) / case.inflow_ratio) for case in trim_conditions]
        fig = plt.figure('ErrorvsVelocity')
        plt.style.use('ggplot')
        plt.plot(velocities, errors)
        plt.title(r'Inflow Ratio Error as a Function of Velocity')
        plt.xlabel(r'True Airspeed, $V_\mathrm{TAS}$ [m/s]')
        plt.ylabel(r'Percentage Error [%]')
        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
        return '%s Plotted and Saved' % fig.get_label()

    @staticmethod
    def plot_trim():
        velocities = np.linspace(0, 150, 100)
        trim_conditions = [Trim(v) for v in velocities]
        pitch = [degrees(case.test_solution[0]) for case in trim_conditions]
        cyclic = [degrees(case.test_solution[1]) for case in trim_conditions]
        fig = plt.figure('TrimvsVelocity')
        plt.style.use('ggplot')
        plt.plot(velocities, pitch, label='Collective')
        plt.plot(velocities, cyclic, label='Cyclic')
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
    print obj.inflow_ratio
    print obj.test_solution
