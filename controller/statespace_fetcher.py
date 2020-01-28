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
    if len(sys.argv) > 1:
        INPUT_VELOCITY = float(sys.argv[1])
    else:
        prompt = '\nPlease Enter a Flight Velocity in m/s: '
        if sys.version_info[0] < 3:  # Adding compatibility for Python 3.x
            INPUT_VELOCITY = str(raw_input(prompt))
        else:
            INPUT_VELOCITY = input(prompt)
    sys.stdout.write(state_wrapper(float(INPUT_VELOCITY)))

