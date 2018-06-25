#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

# http://python-control.readthedocs.io/en/latest/intro.html most likely this will be required

__author__ = ["San Kilkis"]

from globs import Constants, Attribute
from masses import ComponentWeights
from cla_regression import LiftGradient
from ch53_inertia import CH53Inertia

import numpy as np
import copy
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from math import radians, sqrt, pi, degrees, cos, sin, asin, atan
import os  # Necessary to determining the current working directory to save figures

_working_dir = os.getcwd()


class StateSpace(Constants):
    """ Computes the trim condition of the CH-53 based on current velocities

    :param u: Horizontal Flight Velocity in SI meter per second [m/s]
    :type u: float

    :param w: Vertical Flight Velocity in SI meter per second [m/s]
    :type w: float

    :param q: Pitch Rate in SI radian per second [rad/s]
    :type q: float

    :param theta_f: Fuselage Pitch Angle SI radian [rad]
    :type theta_f: float

    :param collective_pitch: Collective Pitch in SI degree [deg]
    :type collective_pitch: float

    :param longitudinal_cyclic: Longitudinal Cyclic in SI degree [deg]
    :type longitudinal_cyclic: float
    """

    def __init__(self, u=0.0, w=0.0, q=0.0, theta_f=0.0, collective_pitch=0.0, longitudinal_cyclic=0.0):
        self.u = float(u)  # Horizontal Velocity [m/s]
        self.w = float(w)  # Vertical Velocity [m/s]
        self.q = float(q)  # Pitch Rate [rad/s]
        self.theta_f = theta_f  # Fuselage Tilt-Angle (Positive up)
        self.collective_pitch = float(collective_pitch)
        self.longitudinal_cyclic = float(longitudinal_cyclic)

    @Attribute
    def velocity(self):
        """ The magnitude of the velocity vector, V,  in trim-condition.

        :return: Velocity in SI meter per second [m/s] """
        return sqrt(self.u**2 + self.w**2)

    @Attribute
    def alpha_control(self):
        """ Computes the Angle of Attack (AoA) of the control plane in SI radian [rad] """
        gamma = atan(self.w / self.u) if self.u != 0.0 else 0
        alpha = self.longitudinal_cyclic - gamma
        return alpha if self.u > 0 else alpha + pi

    @Attribute
    def advance_ratio(self):
        """ Computes the Advance Ratio, also referred to as the tip-speed ratio """
        return (self.velocity * cos(self.alpha_control)) / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def inflow_ratio_control(self):
        """ Computes the Inflow Ratio in the Control Plane """
        return (self.velocity * sin(self.alpha_control)) / (self.main_rotor.omega * self.main_rotor.radius)

    def longitudinal_disk_tilt_func(self, lambda_i):
        """ Computes the longitudinal disk tilt, a1, utilizing an assumed value for the inflow ratio, lambda_i

        :param lambda_i: Inflow Ratio
        :type lambda_i: float
        """
        mu = self.advance_ratio
        theta0 = self.collective_pitch
        lambda_c = self.inflow_ratio_control
        omega = self.main_rotor.omega
        return (((8.0/3.0) * mu * theta0) - (2 * mu * (lambda_c + abs(lambda_i))) -
                ((16.0/self.lock_number) * (self.q / omega))) / (1 - 0.5 * mu**2)

    def thrust_coefficient_elem(self, lambda_i):
        """ Thrust coefficient as defined by the Blade Element Momentum Theory

        :param lambda_i: Inflow Ratio
        :type lambda_i: float
        """
        mu = self.advance_ratio
        cla = self.lift_gradient
        sigma = self.main_rotor.solidity
        lambda_c = self.inflow_ratio_control
        theta0 = self.collective_pitch
        return (0.25 * cla * sigma) * ((2.0/3.0) * theta0 * (1 + (1.5 * mu**2)) - (lambda_c + abs(lambda_i)))

    def thrust_coefficient_glau(self, lambda_i):
        """ Thrust coefficient as defined by the Glauert Theory

        :param lambda_i: Inflow Ratio
        :type lambda_i: float
        """
        a1 = self.longitudinal_disk_tilt_func(lambda_i)
        v = self.velocity
        omega = self.main_rotor.omega
        r = self.main_rotor.radius
        return 2 * abs(lambda_i) * sqrt(((v / (omega * r)) * cos(self.alpha_control - a1))**2 +
                                        ((v / (omega * r)) * sin(self.alpha_control - a1) + abs(lambda_i))**2)

    @Attribute
    def inflow_ratio(self):
        """ Utilizes a numerical solver to compute the inflow ratio as discussed in the lecture slides """

        def func(lambda_i, *args):

            instance, status = args

            return instance.thrust_coefficient_elem(lambda_i) - instance.thrust_coefficient_glau(lambda_i)

        return float(abs((fsolve(func, x0=np.array([1]), args=(self, 'instance_passed')))[0]))

    @Attribute
    def longitudinal_disk_tilt(self):
        return self.longitudinal_disk_tilt_func(self.inflow_ratio)

    @Attribute
    def thrust(self):
        omega = self.main_rotor.omega
        r = self.main_rotor.radius
        return self.thrust_coefficient_elem(self.inflow_ratio) * self.rho * (omega * r)**2 * pi * r**2

    @Attribute
    def rotor_distance_to_cg(self):
        """ Computes the z-axis distance of the main-rotor centroid to the center of gravity (C.G) of the CH-53

        :return: Distance of the Main Rotor to the Center of Gravity (C.G.) on the z-axis in SI meter [m]
        :rtype: float
        """
        inertia_instance = CH53Inertia()
        cg = inertia_instance.get_cg()
        return abs(cg.z - inertia_instance.main_rotor.position.z)

    @Attribute
    def drag(self):
        return self.flat_plate_area*0.5*self.rho*(self.velocity**2)

    @Attribute
    def u_dot(self):
        return -self.g * sin(self.theta_f) - ((self.drag * self.u)/(self.mass_mtow * self.velocity)) + \
               (self.thrust / self.mass_mtow) * sin(self.longitudinal_cyclic - self.longitudinal_disk_tilt) -\
               self.q * self.w

    @Attribute
    def w_dot(self):
        return self.g * cos(self.theta_f) - ((self.drag * self.w)/(self.mass_mtow * self.velocity)) - \
               (self.thrust / self.mass_mtow) * cos(self.longitudinal_cyclic - self.longitudinal_disk_tilt) +\
               self.q * self.u

    @Attribute
    def inertia(self):
        return CH53Inertia().get_inertia()

    @Attribute
    def q_dot(self):
        return (-self.thrust / self.inertia.yy) * self.rotor_distance_to_cg * \
               sin(self.longitudinal_cyclic - self.longitudinal_disk_tilt)

    @Attribute
    def theta_f_dot(self):
        return self.q

    def plot_test(self):

        time = np.linspace(0, 10, 1000)
        delta_t = time[1] - time[0]
        u = [self.u]
        w = [self.w]
        q = [self.q]
        theta_f = [self.theta_f]
        current_case = self
        for i in range(1, len(time)):
            u.append(current_case.u + current_case.u_dot * delta_t)
            w.append(current_case.w + current_case.w_dot * delta_t)
            q.append(current_case.q + current_case.q_dot * delta_t)
            theta_f.append(current_case.theta_f + current_case.theta_f_dot * delta_t)

            # Control Inputs
            if 0.5 < time[i] < 1.0:
                cyclic_input = radians(5.0)
            else:
                cyclic_input = 0.0
            current_case = StateSpace(u=u[i], w=w[i], q=q[i], theta_f=theta_f[i], longitudinal_cyclic=cyclic_input)
            print current_case.thrust
            print current_case.inflow_ratio
        # Plotting Numerical Solution
        fig = plt.figure('TrimvsVelocity')
        plt.style.use('ggplot')

        plt.plot(time, u, label='Horizontal Speed')
        plt.plot(time, w, label='Vertical Speed')

        # Creating Labels & Saving Figure
        plt.title(r'Response of Speeds vs Time.')
        plt.xlabel(r'Velocity [m/s]')
        plt.ylabel(r'Time [s]')
        plt.legend(loc='best')
        plt.show()
        # fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

        return 'Integration Performed'


if __name__ == '__main__':
    obj = StateSpace(u=20.0, w=0.0, q=0, theta_f=radians(0.0), collective_pitch=radians(0.0), longitudinal_cyclic=radians(0.0))
    print obj.inflow_ratio
    print obj.plot_test()