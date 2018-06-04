#  This script will perform the
#
#  Airfoil name is from the 'Redesigned Tail rotor...' document in Mendeley.
#  The Airfoil data is found using XFOIL viscid calculation with the airfoil file from aorfoil tools (flipped)
#  and the reynolds and mach at the mean radius location (R/2). The interpolation returns the C_la.

import os
import matplotlib.pyplot as plt
from scipy import optimize
from math import radians

__author__ = ["San Kilkis", 'Nelson Johnson']

airfoil_file = 'SC1095_data.dat'


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

    @property
    def regression(self):

        def func(x, a, b):
            """ Defines a linear regression function y(x) = a*x + b

            :param x: Value(s) on the x-axis which correspond to the disk_loading
            :type x: int, float, list
            :param a: slope
            :param b: y-intercept """
            return a * x + b

        params, param_covariance = optimize.curve_fit(func, [radians(deg) for deg in self.alpha],
                                                      self.lift_coefficient)
        return params

    @property
    def gradient(self):
        return self.regression[0]

    @property
    def lift_coefficient_alpha0(self):
        return self.regression[1]

    def plot_regression(self):
        fig = plt.figure('LiftGradient')
        plt.style.use('ggplot')
        plt.plot(self.alpha, self.lift_coefficient)
        plt.xlabel(r'Angle of Attack [deg]')
        plt.ylabel(r'Lift Coefficient [-]')
        plt.title('Rotor-Blade Lift Coefficient Gradient')
        plt.show()
        return fig


# r = np.arange(0, 2, 0.01)
# theta = 2 * np.pi * r
#
# fig = plt.figure('BladeAoA')
# plt.style.use('ggplot')
# ax = plt.subplot(111, projection='polar')
# ax.plot(theta, r)
# ax.set_rmax(2)
# ax.set_rticks([0.5, 1, 1.5, 2])  # less radial ticks
# ax.set_rlabel_position(-22.5)  # get radial labels away from plotted line
# ax.grid(True)
#
# ax.set_title("A line plot on a polar axis", va='bottom')
# plt.show()


if __name__ == '__main__':
    obj = LiftGradient()
    obj.plot_regression()








