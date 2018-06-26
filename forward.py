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
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from math import radians, sqrt, pi, degrees, cos, sin, asin
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


class ForwardFlapping(Constants):
    """ Defines the Flapping Dynamics of the CH-53D in Hovering Flight.

    :param collective_pitch: Collective Pitch of the Main Rotor Blades in SI radian [rad]"""

    def __init__(self, collective_pitch=radians(8), lateral_cyclic=radians(1),
                 longitudinal_cyclic=radians(2), velocity=20):
        self.collective_pitch = collective_pitch
        self.lateral_cyclic = lateral_cyclic
        self.longitudinal_cyclic = longitudinal_cyclic
        self.velocity = velocity

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
        """ Utilizes the ACT Definition from Assignment I to calculate the hover induced velocity

        :return: Hover Induced Velocity in SI meter per second [m/s]
        """
        return sqrt(self.weight_mtow/(2*self.rho*pi*(self.main_rotor.radius ** 2)))

    @Attribute
    def disk_aoa(self):
        """ Disk Angle of Attack (AoA) described as the angular distance between the velocity vector and the Tip Path
        Plane (TPP) """
        parasitic_drag = self.flat_plate_area * self.rho * (self.velocity ** 2)
        return asin(parasitic_drag / self.weight_mtow)

    @Attribute
    def induced_velocity(self):
        v_bar = self.velocity / self.hover_induced_velocity

        def func(x):
            """ Defines a 2-th order equation function (V*sin(a) + v_i)**2 + (V*cos(a))**2 - (1/v_i)**2 = 0

            :param x: Represents the Non-Dimensional Induced Velocity
            """
            return ((v_bar * sin(self.disk_aoa) + abs(x)) ** 2 + (v_bar * cos(self.disk_aoa)) ** 2) - (
                    1 / (abs(x) ** 2))

        return abs(fsolve(func, x0=np.array([1]))[0]) * self.hover_induced_velocity

    @Attribute
    def control_aoa(self):
        """ Control Plane (CP) Angle of Attack (AoA) described as the angular distance between the velocity vector and
        the CP """
        alpha_d = self.disk_aoa
        theta_lc = self.longitudinal_cyclic
        theta = self.collective_pitch
        v = self.velocity
        omega = self.main_rotor.omega
        r = self.main_rotor.radius
        lambda_i = self.inflow_ratio

        def func(x):
            mu = (v * cos(x)) / (omega * r)

            return x - alpha_d - theta_lc + (((8. / 3.) * mu * theta)
                                             - 2 * mu * (((v*sin(x))/(omega*r)) + lambda_i)) / (1-(0.5*(mu ** 2)))

        return abs(fsolve(func, x0=np.array([1]))[0])

    @Attribute
    def tip_speed_ratio(self):
        return (self.velocity * cos(self.control_aoa)) / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def inflow_ratio_control(self):
        return (self.velocity * sin(self.control_aoa)) / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def inflow_ratio(self):
        return self.induced_velocity / (self.main_rotor.omega * self.main_rotor.radius)

    @Attribute
    def initial_condition(self):
        return [0, 0]

    def ode(self, x, t):
        """ Defines the blade-flapping velocity and acceleration as a function of the blade-flapping deflection and
        velocity in a system of 1st Order ODEs.

        :param x: The vector of unknown functions that is equal to the [\beta  \dot{\beta}]'
        :param t: Dimensional time in SI seconds [s]
        :return: System of 1st Order ODEs for the Blade Flapping Angle \beta
        :rtype: list
        """

        omega = self.main_rotor.omega  # Renaming the rotational velocity to make the function definition shorter
        lock = self.lock_number  # Renaming the lock number to make the function definition short
        # m_a = self.aerodynamic_moment  # Renaming the non-zero Aerodynamic Moment forcing term
        theta = self.collective_pitch
        r = self.main_rotor.radius
        mu = self.tip_speed_ratio
        lambda_c = self.inflow_ratio_control
        lambda_i = self.inflow_ratio

        b_coef = -1*((omega ** 2) * (1 + (lock / 6.0) * mu * cos(omega * t)
                                        + (lock/8.0) * (mu ** 2) * sin(2*omega*t)))

        b_dot_coef = -1 * ((lock/8.0)*omega*(1+(4.0/3.0) * mu * sin(omega*t)))

        aero_term = (((lock/8.0) * (omega**2) * theta * (1+(mu**2)))
                     - ((lock/6.0) * (omega**2) * (lambda_c + lambda_i))
                     + ((lock/8.0) * (omega**2) * mu * sin(omega * t)
                        * ((8.0/3.0) * theta - (2 * (lambda_c + lambda_i))))
                     - ((lock/8.0) * (omega**2) * theta * (mu**2) * cos(2*omega*t)))

        state_space = [x[1],
                       b_coef * x[0] + b_dot_coef * x[1] + aero_term]

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
    def coning_angle(self):
        """ Returns the Coning Angle SI radians [rad]

        :rtype: float
        """
        lock = self.lock_number
        theta = self.collective_pitch
        mu = self.tip_speed_ratio
        lambda_i = self.inflow_ratio
        lambda_c = self.inflow_ratio_control
        return (lock / 8.0) * (theta * (1+(mu**2))-(4.0/3.0)*(lambda_i+lambda_c))

    @Attribute
    def longitudinal_tilt(self):
        """ Returns the Longitudinal Tilt in SI radians [rad]

        :rtype: float
        """
        theta = self.collective_pitch
        mu = self.tip_speed_ratio
        lambda_i = self.inflow_ratio
        lambda_c = self.inflow_ratio_control
        return (((8.0 / 3.0) * mu * theta) - 2 * mu * (lambda_c + lambda_i)) / (1 - 0.5 * (mu**2))

    @Attribute
    def lateral_tilt(self):
        """ Returns the Longitudinal Tilt in SI radians [rad]

        :rtype: float
        """
        theta_lc = self.lateral_cyclic
        mu = self.tip_speed_ratio
        lambda_i = self.inflow_ratio
        lambda_c = self.inflow_ratio_control
        k_prime = (1.33 * (mu / (lambda_c + lambda_i)))/(1.2 + (mu/(lambda_c + lambda_i)))

        return ((4.0/3.0) * mu * self.coning_angle) / ((1 + 0.5 * (mu**2))
                                                       + (k_prime * lambda_i / (1 + 0.5 * (mu**2)))) + theta_lc

    @Attribute
    def t_final(self):
        """ The time in SI seconds that corresponds to the instance when the advancing blade has completed 1 rev """
        return (2 * pi) / self.main_rotor.omega

    @Attribute
    def t_initial(self):
        return (0 * pi) / self.main_rotor.omega

    @Attribute
    def flap_angle_psi(self):
        """ Returns a fitted-spline for the angular displacement response of the advancing blade in hover for 1 rev """
        time_interval = np.linspace(self.t_initial, self.t_final, 1000)
        rad_interval = [i * self.main_rotor.omega for i in time_interval]
        sol = self.ode_solver(time_interval, ic=self.initial_condition)[0]
        return interp1d(rad_interval, sol, kind='slinear')

    @Attribute
    def flap_velocity_psi(self):
        """ Returns a fitted-spline for the angular velocity response of the advancing blade in hover for 1 rev """
        time_interval = np.linspace(self.t_initial, self.t_final, 1000)
        rad_interval = [i * self.main_rotor.omega for i in time_interval]
        sol = self.ode_solver(time_interval, ic=self.initial_condition)[1]
        return interp1d(rad_interval, sol, kind='slinear')

    def alpha_psi(self, azimuth, radius):
        """ Conputes the Angle of Attack (AoA) of an advancing blade element with the assumption of constant inflow

        :param azimuth: The current azimuth angle of the advancing blade element in SI radian [rad]
        :param radius: The current radius of the advancing blade element in SI meter [m]
        :return: AoA of the advancing blade element in SI [rad]
        """
        return self.collective_pitch - ((self.velocity * sin(self.control_aoa) + self.induced_velocity +
                                         (self.flap_velocity_psi.__call__(azimuth) * radius) + self.velocity
                                         * cos(self.control_aoa) * cos(azimuth)
                                         * sin(self.flap_angle_psi.__call__(azimuth))) /
                                        ((self.main_rotor.omega * radius) + self.velocity * cos(self.control_aoa) *
                                         sin(azimuth)))

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

        levels = [i for i in np.arange(-1, 8, 1.0)]
        cmap = plt.get_cmap('jet')
        cmap_grey = plt.get_cmap('Greys')
        X, Y = np.meshgrid(azimuth, radii)
        # scatter_plot = ax.scatter(X, Y, c=alpha, cmap=cmap)
        contour_plot = ax.contour(X, Y, alpha, cmap=cmap, levels=levels)
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

    def plot_response(self):
        fig = plt.figure('BetaResponse_IC(%1.2f,%1.2f)' % (self.initial_condition[0], self.initial_condition[1]))
        plt.style.use('ggplot')
        gs = gridspec.GridSpec(2, 1, top=0.9)

        # Call to ODE Solver for Plot Solution
        time_interval = np.linspace(self.t_initial, self.t_final * 2, 1000)
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
    obj = ForwardFlapping(velocity=20)
    obj.plot_response()

    # print 'Con. Angle = ' + str(degrees(obj.coning_angle))
    # print 'Long. Tilt = ' + str(degrees(obj.longitudinal_tilt))
    # print 'Lat. Tilt = ' + str(degrees(obj.lateral_tilt))
    #
    # print 'Con. Angle = ' + str(obj.coning_angle)
    # print 'Long. Tilt = ' + str(obj.longitudinal_tilt)
    # print 'Lat. Tilt = ' + str(obj.lateral_tilt)
