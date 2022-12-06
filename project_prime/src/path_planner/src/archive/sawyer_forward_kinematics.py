#!/usr/bin/env python

import numpy as np
import kin_func_skeleton as kfs

def sawyer_forward_kinematics_from_angles(joint_angles):

    # xi's at the zero configuration
    xi_stacked = np.array([[-0.00000000e+00, -3.16242120e-01,  1.16080254e-03,
            -3.15726565e-01,  1.61240502e-03, -3.14144466e-01,
             9.20890217e-04],
           [-0.00000000e+00,  1.98024061e-03,  3.17245332e-01,
             4.62386380e-03,  3.20134825e-01,  4.20912799e-03,
             3.13764870e-01],
           [-0.00000000e+00,  8.07487331e-02, -1.92571890e-01,
             4.81956263e-01, -2.34330995e-02,  8.80735090e-01,
            -1.56793991e-01],
           [ 0.00000000e+00,  4.98496037e-03,  9.99958989e-01,
             6.97880485e-03,  9.99947986e-01,  4.99895605e-03,
             9.99996004e-01],
           [ 0.00000000e+00,  9.99975075e-01, -7.08396226e-03,
             9.99963038e-01, -5.65756149e-03,  9.99983017e-01,
            -2.81714342e-03],
           [ 1.00000000e+00, -4.99991526e-03, -5.64256299e-03,
            -5.02182250e-03, -8.48634223e-03, -2.99597706e-03,
             2.35760938e-04]])

    init_homog = np.array([
        [-0.0027528, 0.0053175, 0.9999821, 0.882], 
        [-0.0008212, 0.9999855, -0.0053197, 0.158], 
        [-0.9999959, -0.0008359, -0.0027484, 0.313], 
        [0, 0, 0, 1]
    ])
    return kfs.prod_exp(xi_stacked, joint_angles) @ init_homog


def get_position_from_fwd_kinematics(joint_angles):
    g_st_theta =  sawyer_forward_kinematics_from_angles(joint_angles)
    position_vector = g_st_theta[:,2][:2]