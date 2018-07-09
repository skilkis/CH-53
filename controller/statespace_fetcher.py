#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the wrapper interface to be able to instantiate the :class:`StateSpace` from MATLAB and retrieve
the linearized EoM """

__author__ = ["San Kilkis"]

import __root__
import sys
from model import StateSpace
import scipy.io
assert __root__  # Necessary to circumvent PEP-8 Syntax violation on the __root__ import statement


def state_wrapper(initial_velocity):

    print ('\nLinearizing System Dynamics at V = %1.4f [m/s] \n' % initial_velocity)

    ss_obj = StateSpace(initial_velocity)
    scipy.io.savemat('ss.mat', dict(A=ss_obj.a_matrix, B=ss_obj.b_matrix, C=ss_obj.c_matrix, D=ss_obj.d_matrix,
                                    velocity=initial_velocity,
                                    u=ss_obj.initial_trim_case.u,
                                    w=ss_obj.initial_trim_case.w,
                                    thetaf=ss_obj.initial_trim_case.fuselage_tilt,
                                    thetac=ss_obj.initial_trim_case.longitudinal_cyclic,
                                    theta0=ss_obj.initial_trim_case.collective_pitch))
    return 'Operation Successful'


if __name__ == '__main__':
    input_velocity = float(sys.argv[1])
    sys.stdout.write(state_wrapper(input_velocity))
