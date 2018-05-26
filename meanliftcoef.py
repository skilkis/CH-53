#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Şan Kılkış"

""" A file containing an optimized linear least-squares regression in order to estimate the mean lift coefficient """

from scipy import optimize
import matplotlib.pyplot as plt
import numpy as np
import os
from globs import DL


def medium_lift_coef(disk_loading=350, plot=True):
    """ A function that allows the user to quickly determine an estimate for the Medium Lift Coefficient based on a disk
    loading for a given helicopter.

    :param disk_loading: The disk loading value of the current helicopter model in SI Newton per meter squared [N/m^2]
    :param plot: A boolean to turn on/off the plotting functionality
    :return: Estimated Medium Lift Coefficient

    Usage:

    >>> lift_coefficent = medium_lift_coef(disk_loading=490)
    >>> print lift_coefficent
    0.695935841011
    """

    disk_loading_data = np.array([262.3, 309.6, 363.2, 428.6, 325.1])
    mean_lift_coefficient_data = np.array([0.485, 0.499, 0.552, 0.647, 0.556])

    def func(x, a, b):
        """ Defines a linear regression function y(x) = a*x + b

        :param x: Value(s) on the x-axis which correspond to the disk_loading
        :type x: int, float, list
        :param a: slope
        :param b: y-intercept """
        return a*x + b

    params, param_covariance = optimize.curve_fit(func, disk_loading_data, mean_lift_coefficient_data)

    # Switch-case to allow for accessing the function w/o plotting
    if plot:
        _working_dir = os.getcwd()  # Added for save functionality
        fig = plt.figure('MediumLiftCoeficient')
        plt.style.use('ggplot')
        plt.title('Medium Lift Coefficient as a Function of Disk Loading')

        # Scatter Plot of Data
        plt.scatter(disk_loading_data, mean_lift_coefficient_data, label='Data')

        # Calculating R^2
        residuals = mean_lift_coefficient_data - func(disk_loading_data, params[0], params[1])
        ss_res = np.sum(residuals ** 2)
        ss_tot = np.sum((mean_lift_coefficient_data - np.mean(mean_lift_coefficient_data)) ** 2)
        r_squared = float(1 - (ss_res / ss_tot))

        # Linear-Regression Plot
        xvals = np.linspace(200, 500, 10)
        plt.plot(xvals, func(xvals, params[0], params[1]),
                 label=r'Linear Regression:' +
                       '\n' r'$\bar{C}_L=%f\cdot\mathrm{DL} + %f$' % (params[0], params[1]) +
                       '\n' r'$R^2$ = %0.2f' % r_squared)

        plt.plot(disk_loading, func(disk_loading, params[0], params[1]),
                 marker='o',
                 markerfacecolor='white',
                 markeredgecolor='black', markeredgewidth=1,
                 linewidth=0,
                 label=r'Disk Loading = %0.1f [N/m$^2$]' % disk_loading)

        plt.xlabel(r'Disk Loading [N/m$^2$]')
        plt.ylabel(r'Medium Lift Coefficient [-]')
        plt.legend(loc='best')
        plt.ion()
        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')

    return func(disk_loading, params[0], params[1])


if __name__ == '__main__':
    f_call = medium_lift_coef(DL)
    print f_call

