#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to estimate the Mass Moment of Inertia of the CH53 Helicopter """

__author__ = ["San Kilkis"]

from globs import Constants
from masses import ComponentWeights
from cla_regression import LiftGradient

import numpy as np
from scipy import integrate
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from math import radians, sqrt, pi, degrees, cos, sin
from basic_units import radians as rad_ticks  # Renaming to remove conflict with built-in package
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


class HoverFlapping(Constants):
    """ Defines the Flapping Dynamics of the CH-53D in Hovering Flight.

    :param collective_pitch: Collective Pitch of the Main Rotor Blades in SI radian [rad]"""

    def __init__(self, collective_pitch=radians(8)):
        self.collective_pitch = collective_pitch

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
    def hover_induced_velocity(self):
        """

        :return: Hover Induced Velocity in SI meter per second [m/s]
        """
        return sqrt(self.weight_mtow/(2*self.rho*pi*(self.main_rotor.radius ** 2)))

    @Attribute
    def inflow_ratio(self):
        return self.hover_induced_velocity / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def coning_angle(self):
        """ This parameter presents the steady-state particular solution which is simply a constant value """
        return (self.lock_number / 8.0) * (self.collective_pitch - ((4.0 * self.inflow_ratio) / 3.0))

    @Attribute
    def aerodynamic_moment(self):
        return self.coning_angle * (self.main_rotor.omega ** 2)

    @Attribute
    def initial_condition(self):
        return [0, 0]

    def ode(self, x, t):
        """ Defines the blade-flapping velocity and acceleration as a function of the blade-flapping deflection and
        velocity in a system of 1st Order ODEs.

        :param x: The state-space vector that is equal to the [\beta  \dot{\beta}]'
        :param t: Dimensional time in SI seconds [s]
        :return: System of 1st Order ODEs for the Blade Flapping Angle \beta
        :rtype: list
        """

        omega = self.main_rotor.omega  # Renaming the rotational velocity to make the function definition shorter
        lock = self.lock_number  # Renaming the lock number to make the function definition short
        m_a = self.aerodynamic_moment  # Renaming the non-zero Aerodynamic Moment forcing term

        state_space = [x[1],
                       -1*((omega ** 2) * x[0]) - ((lock / 8.0) * omega * x[1]) + m_a]

        return state_space

    def ode_solver(self, t, **kwargs):
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

    @Attribute
    def t_final(self):
        """ The time in SI seconds that corresponds to the instance when the advancing blade has completed 1 rev """
        return (2 * pi) / self.main_rotor.omega

    @Attribute
    def flap_velocity_psi(self):
        """ Returns a fitted-spline for the angular velocity response of the advancing blade in hover for 1 rev """
        time_interval = np.linspace(0, self.t_final, 1000)
        rad_interval = [i * self.main_rotor.omega for i in time_interval]
        sol = self.ode_solver(time_interval, ic=self.initial_condition)[1]
        return interp1d(rad_interval, sol, kind='slinear')

    def alpha_psi(self, azimuth, radius):
        """ Conputes the Angle of Attack (AoA) of an advancing blade element with the assumption of constant inflow

        :param azimuth: The current azimuth angle of the advancing blade element in SI radian [rad]
        :param radius: The current radius of the advancing blade element in SI meter [m]
        :return: AoA of the advancing blade element in SI [rad]
        """
        return self.collective_pitch - ((self.hover_induced_velocity + (self.flap_velocity_psi.__call__(azimuth) *
                                                                        radius)) / (self.main_rotor.omega * radius))

    def plot_alpha(self):
        azimuth = np.linspace(0, 2*pi, 360)
        radii = np.linspace(3, self.main_rotor.radius, 60)

        fig = plt.figure('BladeAoA')
        plt.style.use('ggplot')
        ax = plt.subplot(111, projection='polar')

        alpha = np.zeros((len(radii), len(azimuth)))
        for i in range(0, len(radii)):
            r = radii[i]
            for j in range(0, len(azimuth)):
                phi = azimuth[j]
                alpha[i, j] = degrees(self.alpha_psi(phi, r))

        cmap = plt.get_cmap('jet')
        cmap_grey = plt.get_cmap('Greys')
        X, Y = np.meshgrid(azimuth, radii)
        # scatter_plot = ax.scatter(X, Y, c=alpha, cmap=cmap)
        contour_plot = ax.contour(X, Y, alpha, cmap=cmap, vmin=-6, vmax=5)
        ax.set_rlabel_position(-22.5)  # get radial labels away from plotted line
        ax.set_theta_zero_location("S")
        ax.set_rlim(0, self.main_rotor.radius)
        ax.grid(True)
        plt.clabel(contour_plot, inline=1, fontsize=10)
        # plt.colorbar(scatter_plot, shrink=0.8, extend='both')

        ax.set_title("Adv. Blade Element AoA vs. Azimuth and Radius", va='bottom')
        ax.set_xlabel(r'Blade Azimuth $\psi$ [rad]')
        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
        return 'Plot Created and Saved'

    # def plot_flapangle(self):
    #     time_interval = np.linspace(0, 1, 1000)
    #     fig = plt.figure('psivsBeta')
    #     plt.style.use('ggplot')
    #     ax = fig.gca()
    #     sol = self.ode_solver(time_interval, ic=self.initial_condition)[0]
    #     blade_position = [(t * self.main_rotor.omega) * rad_ticks for t in time_interval]
    #     ax.plot(blade_position, sol, xunits=rad_ticks)
    #     plt.title(r'Blade Angular Displacement Response $\beta_0, \dot{\beta}_0 = \left(0, 0\right)$')
    #     plt.xlabel(r'Blade Azimuth $\psi$ [rad]')
    #     plt.ylabel(r'Blade Flapping Angle  $\beta_0 = 0$ [rad]')
    #     plt.show()
    #     fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
    #     return 'Plot Created and Saved'
    #
    # def plot_bladevelocity(self):
    #     time_interval = np.linspace(0, 1, 1000)
    #     fig = plt.figure('psivsBeta')
    #     plt.style.use('ggplot')
    #     ax = fig.gca()
    #     sol = self.ode_solver(time_interval, ic=self.initial_condition)[1]
    #     blade_position = [(t * self.main_rotor.omega) * rad_ticks for t in time_interval]
    #     ax.plot(blade_position, sol, xunits=rad_ticks)
    #     plt.title(r'Blade Angular Velocity Response $\beta_0, \dot{\beta}_0 = \left(0, 0\right)$')
    #     plt.xlabel(r'Blade Azimuth $\psi$ [rad]')
    #     plt.ylabel(r'Blade Flapping Velocity  $\beta_0 = 0$ [rad/s]')
    #     plt.show()
    #     fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
    #     return 'Plot Created and Saved'

    def plot_response(self):
        fig = plt.figure('BetaResponse_IC(%1.2f,%1.2f)' % (self.initial_condition[0], self.initial_condition[1]))
        plt.style.use('ggplot')
        gs = gridspec.GridSpec(2, 1, top=0.9)

        # Call to ODE Solver for Plot Solution
        time_interval = np.linspace(0, self.t_final * 2, 1000)
        sol = self.ode_solver(time_interval, ic=self.initial_condition)
        blade_position = [(t * self.main_rotor.omega) * rad_ticks for t in time_interval]

        # Displacement Plot
        top_plot = fig.add_subplot(gs[0, 0])
        top_plot.plot(blade_position, sol[0], xunits=rad_ticks)
        top_plot.set_ylabel(r'$\beta$ [rad]')
        top_plot.set_xlabel('')

        # Velocity Plot
        bot_plot = fig.add_subplot(gs[1, 0])
        bot_plot.plot(blade_position, sol[1], xunits=rad_ticks)
        bot_plot.set_xlabel(r'Blade Azimuth $\psi$ [rad]')
        bot_plot.set_ylabel(r'$\dot{\beta}$ [rad\s]')

        # Figure Title
        plt.suptitle(r'Blade Angular Response $\beta_0, \dot{\beta}_0 = \left(%1.2f, %1.2f\right)$'
                     % (self.initial_condition[0], self.initial_condition[1]))

        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')


if __name__ == '__main__':
    obj = HoverFlapping()

    # obj.plot_flapangle()
    obj.plot_alpha()
    obj.plot_response()
    # print obj.inflow_ratio
    # print obj.coning_angle
    # print obj.lock_number
