#!/usr/bin/env python
"""
Kinematic function skeleton code for Lab 3 prelab.
Course: EE 106A, Fall 2021
Originally written by: Aaron Bestick, 9/10/14
Adapted for Fall 2020 by: Amay Saxena, 9/10/20
This Python file is a code skeleton for Lab 3 prelab. You should fill in 
the body of the five empty methods below so that they implement the kinematic 
functions described in the assignment.
When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.
"""

import numpy as np

np.set_printoptions(precision=4,suppress=True)

#-----------------------------2D Examples---------------------------------------
#--(you don't need to modify anything here but you should take a look at them)--

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """
    
    rot = np.zeros((2,2))
    rot[0,0] = np.cos(theta)
    rot[1,1] = np.cos(theta)
    rot[0,1] = -np.sin(theta)
    rot[1,0] = np.sin(theta)

    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    xi_hat = np.zeros((3,3))
    xi_hat[0,1] = -xi[2]
    xi_hat[1,0] =  xi[2]
    xi_hat[0:2,2] = xi[0:2]

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    g = np.zeros((3,3))
    wtheta = xi[2]*theta
    R = rotation_2d(wtheta)
    p = np.dot(np.dot( \
        [[1 - np.cos(wtheta), np.sin(wtheta)],
        [-np.sin(wtheta), 1 - np.cos(wtheta)]], \
        [[0,-1],[1,0]]), \
        [[xi[0]/xi[2]],[xi[1]/xi[2]]])

    g[0:2,0:2] = R
    g[0:2,2:3] = p[0:2]
    g[2,2] = 1

    return g

#-----------------------------3D Functions--------------------------------------
#-------------(These are the functions you need to complete)--------------------

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """

    wx = omega[0]

    wy = omega[1]

    wz = omega[2]


    omega_hat = np.zeros((3,3))


    omega_hat[0][1] = -wz

    omega_hat[0][2] = wy

    omega_hat[1][0] = wz

    omega_hat[1][2] = -wx

    omega_hat[2][0] = -wy

    omega_hat[2][1] = wx


    return omega_hat


def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """

    norm_w = np.linalg.norm(omega)

    omega_hat = skew_3d(omega)

    

    term_1 = np.eye(3)

    

    term_2 = (1/norm_w)*omega_hat*np.sin(norm_w*theta)

    

    term_3 = omega_hat@omega_hat*(1/norm_w)*(1/norm_w)*(1-np.cos(norm_w*theta))

    

    return term_1 + term_2 + term_3


def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """

    v = xi[0:3]

    w = xi[3:]


    w_hat = skew_3d(w)

    v = v.reshape((len(v), 1))

    xi_hat = np.hstack((w_hat, v))

    xi_hat = np.vstack((xi_hat, np.zeros((1,4))))

    return xi_hat


def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement
    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """

    v = xi[0:3]

    v = v.reshape((len(v), 1))

    w = xi[3:]


    if not any([w[0], w[1], w[2]]):

        # In this case, w=0

        g = np.hstack((np.eye(3), v*theta))

        g = np.vstack((g, np.array([0, 0, 0, 1])))

        print(g)

        return g

    else:

        top_left = rotation_3d(w, theta)

        w_hat = skew_3d(w)

        a = 1/(np.inner(w,w))

        b = (np.eye(3)-top_left)@(w_hat@v)

        c = np.outer(w,w)

        c = c@(v*theta)


        top_right = a*(b+c)

        g = np.hstack((top_left, top_right))

        g = np.vstack((g, np.array([0, 0, 0, 1])))

        return g



    


def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6, N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """

    _, N = np.shape(xi)


    g = np.eye(4)


    for i in range(N):

        x = xi.T[i]

        theta_i = theta[i]

        trans_i = homog_3d(x, theta_i)


        g = g@trans_i


    return g