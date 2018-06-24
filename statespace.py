#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to calculate the trim conditions of the CH53 Helicopter """

__author__ = ["San Kilkis"]

from globs import Constants
from masses import ComponentWeights
from cla_regression import LiftGradient

import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from math import radians, sqrt, pi, degrees, cos, sin, asin, atan
import os  # Necessary to determining the current working directory to save figures

_working_dir = os.getcwd()


class Attribute(object):
    """ A decorator that is used for lazy evaluation of an object attribute.
    property should represent non-mutable data, as it replaces itself. """

    def __init__(self, fget):
        self.fget = fget
        self.func_name = fget.__name__

    def __get__(self, obj, cls):
        if obj is None:
            return None
        value = self.fget(obj)
        setattr(obj, self.func_name, value)
        return value


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
    def weights(self):
        """ Instantiates the Weight Estimating Relationships class to be accessed by the rest of the class

        :return: Class containing all Component Weights
        """
        return ComponentWeights()

    @Attribute
    def inertia_blade(self):
        """ Single blade Mass Moment of Inertia about the flapping hinge assuming that the blade length runs from
        hub-center to tip.

        :return: Mass Moment of Inertia in SI kilogram meter squared [kg m^2]
        """
        return (1.0/3.0) * \
               (self.weights.kg_to_lbs(self.weights.W_2A, power=-1) /
                self.main_rotor.blade_number) * self.main_rotor.radius**2

    @Attribute
    def velocity(self):
        """ The magnitude of the velocity vector, V,  in trim-condition.

        :return: Velocity in SI meter per second [m/s] """
        return sqrt(self.u**2 + self.w**2)

    @Attribute
    def lift_gradient(self):
        """ Lift coefficient gradient of the CH-53D main rotor (SC1095 Airfoil)

        :return: Lift Coefficient Gradient in SI one over radians [1/rad]
        """
        return LiftGradient().gradient

    @Attribute
    def lock_number(self):
        """ Represents the ratio of aerodynamic excitation forces to the inertial forces on the blade

        :return: Non-Dimensional Lock Number [-]
        """
        return (self.rho * self.lift_gradient * self.main_rotor.chord * (self.main_rotor.radius ** 4)) \
               / self.inertia_blade

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
    # TODO This doesn't work
    def inflow_ratio(self):
        """ Utilizes a numerical solver to compute the inflow ratio as discussed in the lecture slides """

        def func(lambda_i):
            return self.thrust_coefficient_elem(lambda_i) - self.thrust_coefficient_glau(lambda_i)

        return float(abs((fsolve(func, x0=np.array([1])))[0]))

    @Attribute
    def longitudinal_disk_tilt(self):
        return self.longitudinal_disk_tilt_func(self.inflow_ratio)

    @Attribute
    def thrust(self):
        omega = self.main_rotor.omega
        r = self.main_rotor.radius
        return self.thrust_coefficient_elem(self.inflow_ratio) * self.rho * (omega * r)**2 * pi * r**2

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
    def q_dot(self):
        # TODO Change the 1.0 to the correct height of the disk plane from the c.g.
        return (-self.thrust / self.inertia_blade) * 1.0 * sin(self.longitudinal_cyclic - self.longitudinal_disk_tilt)

    @Attribute
    def theta_f_dot(self):
        return self.q


if __name__ == '__main__':
    obj = StateSpace(0, 0)
    print obj.inflow_ratio
    print obj.longitudinal_disk_tilt
    print obj.thrust_coefficient_elem(obj.inflow_ratio)
    print obj.thrust
    print obj.drag
    print obj.u_dot
    print obj.w_dot
    print obj.q_dot
