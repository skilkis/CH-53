#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the wrapper interface to be able to instantiate the 
:class:`StateSpace` from MATLAB and retrieve the linearized EoM """

from scipy.io import savemat

if __package__:
    from ..model import StateSpace
else:
    import sys
    sys.path.insert(0, '..')
    from model.statespace import StateSpace

__author__ = ["San Kilkis"]


def state_wrapper(initial_velocity):

    print ('\nLinearizing System Dynamics at V = %1.4f [m/s] \n' % initial_velocity)

    ss_obj = StateSpace(initial_velocity)
    savemat('ss.mat',
            dict(A=ss_obj.a_matrix,
                 B=ss_obj.b_matrix,
                 C=ss_obj.c_matrix,
                 D=ss_obj.d_matrix,
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
