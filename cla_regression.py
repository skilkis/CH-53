#  This script will perform the
#
#  Airfoil name is from the 'Redesigned Tail rotor...' document in Mendeley.
#  The Airfoil data is found using XFOIL viscid calculation with the airfoil file from aorfoil tools (flipped)
#  and the reynolds and mach at the mean radius location (R/2). The interpolation returns the C_la.

import os
import matplotlib.pyplot as plt
from scipy import optimize
from math import radians

filename = 'SC1095_data.dat'


class LiftGradient(object):

    def __init__(self, filename='SC1095_data.dat'):
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
        lines = data.readlines()[12:]
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
        params, param_covariance = optimize.curve_fit(self.func, [radians(deg) for deg in self.alpha], self.lift_coefficient)
        return params

    @property
    def gradient(self):
        return self.regression[0]

    @property
    def lift_coefficient_alpha0(self):
        return self.regression[1]


if __name__ == '__main__':
    obj = LiftGradient()
    print obj








