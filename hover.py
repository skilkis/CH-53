#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to estimate the Mass Moment of Inertia of the CH53 Helicopter """

__author__ = ["San Kilkis"]

from globs import Constants
from masses import ComponentWeights
from cla_regression import LiftGradient

import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
from math import radians, sqrt, pi
from basic_units import radians as rad_ticks  # Renaming to remove conflict with built-in package


class HoverFlapping(Constants):

    def __init__(self, collective_pitch=radians(8)):
        self.collective_pitch = collective_pitch

    @property
    def weights(self):
        """ Intantiates the Weight Estimating Relationships class to be accessed by the rest of the class

        :return: Class containing all Component Weights
        """
        return ComponentWeights()

    @property
    def inerta_blade(self):
        """ Single blade Mass Moment of Inertia about the flapping hinge assuming that the blade length runs from
        hub-center to tip.

        :return: Mass Moment of Inertia in SI kilogram meter squared [kg m^2]
        """
        return (1.0/3.0) * \
               (self.weights.kg_to_lbs(self.weights.W_2A, power=-1) /
                self.main_rotor.blade_number) * self.main_rotor.radius**2

    @property
    def lift_gradient(self):
        return LiftGradient().gradient

    @property
    def lock_number(self):
        return (self.rho * self.lift_gradient * self.main_rotor.chord * (self.main_rotor.radius ** 4)) \
               / self.inerta_blade

    @property
    def inflow_ratio(self):
        v_i_hover = sqrt(self.weights.W_g_kg/(2*self.rho*pi*(self.main_rotor.radius ** 2)))
        return v_i_hover / (self.main_rotor.omega * self.main_rotor.radius)

    @property
    def aerodynamic_moment(self):
        return self.coning_angle * self.main_rotor.omega ** 2

    def ode(self, x, t):
        """ Defines the blade-flapping velocity and acceleration as a function of the blade-flapping deflection and
        velocity in state-space form.

        :param x: The state-space vector that is equal to the [\beta  \dot{\beta}]'
        :param t: Dimensional time in SI seconds [s]
        :return: State-Space Model for the Blade Flapping Angle \beta
        :rtype: list
        """

        omega = self.main_rotor.omega  # Renaming the rotational velocity to make the function definition shorter
        lock = self.lock_number  # Renaming the lock number to make the function definition short
        m_a = self.aerodynamic_moment  # Renaming the non-zero Aerodynamic Moment forcing term

        state_space = [x[1],
                       -1*((omega ** 2) * x[0]) - ((lock / 8.0) * omega * x[1]) + m_a]

        return state_space

    def ode_solution(self, t, **kwargs):
        """ Solves the differential equation in state-space form defined by `ode_statepace` with the lsoda package
        from the FORTRAN library odepack. Assumed initial values are a blade deflection \beta = 0 [rad] and blade
        velocity \dot{\beta} = 0

        :param t: Time interval for which the solution is desired
        :type t: numpy.ndarray
        :param ic: Initial Conditions of the system, if unspecified [0, 0] is used
        :type ic: list
        """
        ic = kwargs.get('ic') if 'ic' in kwargs.keys() else [0, 0]  # Fetching initial condition

        sol = integrate.odeint(self.ode, ic, t)
        beta = sol[:, 0]
        beta_dot = sol[:, 1]

        return beta, beta_dot

    @property
    def coning_angle(self):
        """ This parameter presents the steady-state particular solution which is simply a constant value """
        return (self.lock_number / 8.0) * (self.collective_pitch - ((4.0 * self.inflow_ratio) / 3.0))

    @property
    def t_final(self):
        """ The time in SI seconds that corresponds to the instance when the advancing blade has completed 1 rev """
        return (2 * pi) / self.main_rotor.omega

    def plot_homogeneous(self):
        time_interval = np.linspace(0, 1, 1000)
        fig = plt.figure('PsivsBeta')
        plt.style.use('ggplot')
        ax = fig.gca()
        sol = self.ode_solution(time_interval)[0]
        blade_position = [(t * self.main_rotor.omega) * rad_ticks for t in time_interval]
        ax.plot(blade_position, sol, xunits=rad_ticks)
        plt.xlabel(r'Blade Azimuth $\Psi$ [rad]')
        plt.ylabel(r'Non-Dimensional Flapping Angle  $\frac{\beta}{\beta_0}$')
        plt.show()

    def plot_bladeangle(self):
        time_interval = np.linspace(0, 1, 1000)
        fig = plt.figure('PsivsBeta')
        plt.style.use('ggplot')
        ax = fig.gca()
        sol = self.ode_solution(time_interval)[0]
        blade_position = [(t * self.main_rotor.omega) * rad_ticks for t in time_interval]
        ax.plot(blade_position, sol, xunits=rad_ticks)
        plt.xlabel(r'Blade Azimuth $\Psi$ [rad]')
        plt.ylabel(r'Blade Flapping Angle  $\beta_0 = 0$ [rad]')
        plt.show()

    def plot_bladevelocity(self):
        time_interval = np.linspace(0, 1, 1000)
        fig = plt.figure('PsivsBeta')
        plt.style.use('ggplot')
        ax = fig.gca()
        sol = self.ode_solution(time_interval)[1]
        blade_position = [(t * self.main_rotor.omega) * rad_ticks for t in time_interval]
        ax.plot(blade_position, sol, xunits=rad_ticks)
        plt.xlabel(r'Blade Azimuth $\Psi$ [rad]')
        plt.ylabel(r'Blade Flapping Velocity  $\beta_0 = 0$ [rad/s]')
        plt.show()


if __name__ == '__main__':
    obj = HoverFlapping()
    # obj.test()
    obj.plot_bladeangle()
    obj.plot_bladevelocity()
    print obj.coning_angle
    print obj.lock_number
