#  This script will perform the
#
#  Airfoil name is from the 'Redesigned Tail rotor...' document in Mendeley.
#  The Airfoil data is found using XFOIL viscid calculation with the airfoil file from airfoil tools (flipped)
#  and the reynolds and mach at the mean radius location (R/2). The interpolation returns the C_la.

import os
import matplotlib.pyplot as plt
from scipy import optimize
from math import radians, degrees
import numpy as np

__author__ = ["San Kilkis", 'Nelson Johnson']

airfoil_file = 'SC1095_data.dat'
_working_dir = os.getcwd()


class LiftGradient(object):

    def __init__(self, filename=airfoil_file):
        self.filename = filename

    def __repr__(self):
        return 'Lift Gradient = %2.5f Lift Coefficient at Alpha 0 = %2.5f' % (self.gradient,
                                                                              self.lift_coefficient_alpha0)

    @property
    def alpha(self):
        return self.read_xfoil_data()[0]

    @property
    def lift_coefficient(self):
        return self.read_xfoil_data()[1]

    def read_xfoil_data(self):
        data = open(os.path.join(os.getcwd(), self.filename))
        lines = data.readlines()[12:]  # Skipping Header
        data.close()

        alpha = []
        lift_coefficient = []
        for line in lines:
            filtered_line = line.replace('\n', '')
            row = filtered_line.split('  ')
            row_filtered = [float(i) for i in row if i.replace(' ', '') is not '']
            alpha.append(row_filtered[0])
            lift_coefficient.append(row_filtered[1])

        return alpha, lift_coefficient

    @staticmethod
    def func(x, a, b):
        """ Defines a linear regression function y(x) = a*x + b

        :param x: Value(s) on the x-axis which correspond to the disk_loading
        :type x: int, float, list
        :param a: slope
        :param b: y-intercept """
        return a * x + b

    @property
    def regression(self):

        params, param_covariance = optimize.curve_fit(self.func, [radians(deg) for deg in self.alpha],
                                                      self.lift_coefficient)
        return params, param_covariance

    @property
    def gradient(self):
        return self.regression[0][0]

    @property
    def lift_coefficient_alpha0(self):
        return self.regression[0][1]

    def plot_regression(self):
        fig = plt.figure('LiftGradient')
        plt.style.use('ggplot')
        plt.scatter(self.alpha, self.lift_coefficient, label='Data')

        gradient = self.gradient / degrees(1)
        cl_0 = self.lift_coefficient_alpha0
        xvals = np.linspace(-10, 10, len(self.read_xfoil_data()[0]))

        # Calculating R^2
        residuals = np.array(self.read_xfoil_data()[1]) - self.func(xvals, gradient, cl_0)
        ss_res = np.sum(residuals ** 2)
        ss_tot = np.sum((np.array(self.read_xfoil_data()[1]) - np.mean(np.array(self.read_xfoil_data()[1]))) ** 2)
        r_squared = float(1 - (ss_res / ss_tot))

        plt.plot(xvals, self.func(xvals, gradient, cl_0),
                 label=r'Linear Regression:' +
                       '\n' r'$\bar{C}_L=%f\cdot\mathrm{\alpha} + %f$' % (gradient, cl_0) +
                       '\n' r'$R^2$ = %0.2f' % r_squared)



        plt.xlabel(r'Angle of Attack [deg]')
        plt.ylabel(r'Lift Coefficient [-]')
        plt.title('Rotor-Blade Lift Coefficient Gradient')
        plt.legend(loc='best')
        plt.ion()
        plt.show()
        fig.savefig(fname=os.path.join(_working_dir, 'Figures', '%s.pdf' % fig.get_label()), format='pdf')
        return fig


if __name__ == '__main__':
    obj = LiftGradient()
    obj.plot_regression()








