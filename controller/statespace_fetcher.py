import sys
from statespace import StateSpace
import scipy.io


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
