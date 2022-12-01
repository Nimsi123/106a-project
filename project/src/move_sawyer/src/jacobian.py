# ./intera.sh amir.local

from tf.transformations import quaternion_matrix
import numpy as np

# computes the manipulator Jacobian for Sawyer

# use RViz to command Sawyer to 0 configuration
# use RViz to read off q and w values, and thus find the xi's and the corresponding Jacobian matrix

right_l0 = {"position": [0, 0, 0.08], "orientation": [0, 0, 0.003, 1]}
right_l1 = {"position": [0.081, 0.05, 0.316], "orientation": [-0.5, 0.504, 0.499, 0.5]}
right_l2 = {"position": [0.223, 0.191, 0.316], "orientation": [0.001, 0.709, -0.004, 0.705]}
right_l3 = {"position": [0.483, 0.147, 0.315], "orientation": [-0.498, 0.505, 0.499, 0.499]}
right_l4 = {"position": [0.607, 0.02, 0.315], "orientation": [0.002, 0.71, -0.002, 0.704]}
right_l5 = {"position": [0.881, 0.05, 0.314], "orientation": [-0.5, 0.502, 0.498, 0.501]}
right_l6 = {"position": [0.992, 0.154, 0.314], "orientation": [0.705, 0.058, 0.705, 0.06]}
right_hand = {"position": [1.017, 0.154, 0.314], "orientation": [0.539, -0.457, 0.540, -0.456]}

revolute_joints = [right_l0, right_l1, right_l2, right_l3, right_l4, right_l5, right_l6]

# preprocess
for joint in revolute_joints:
	# arrange values to [q.w, q.x, q.y, q.z]
	q_z = joint["orientation"][3]
	joint["orientation"][1:3] = joint["orientation"][0:2]
	joint["orientation"][0] = q_z


manipulator_jacobian = np.zeros((6, 7))
for i, joint in enumerate(revolute_joints):
	q = joint["position"]
	R_SE3 = quaternion_matrix(joint["orientation"])
	w = R_SE3[:3, 2] # z axis is the axis of rotation
	v = - np.cross(w, q)
	xi = np.concatenate((v, w))
	manipulator_jacobian[:, i] = xi

print(repr(manipulator_jacobian))

"""
np.array([[ 8.00000000e-02,  3.16194961e-01,  2.73718737e-01,
         3.16015605e-01,  2.14951957e-01,  3.14098549e-01,
        -1.95910301e-01],
       [-0.00000000e+00,  9.34949244e-04,  1.36463949e-01,
        -1.18932680e-03,  6.55368961e-03, -1.13173427e-03,
         1.02141999e+00],
       [-0.00000000e+00, -8.11979092e-02, -2.75645230e-01,
        -4.84002242e-01, -4.14625117e-01, -8.81096290e-01,
         1.17975606e-01],
       [ 0.00000000e+00,  3.98400025e-03,  6.68857217e-01,
         6.97880485e-03,  6.70415211e-01,  1.99600401e-03,
         1.80382160e-01],
       [-1.00000000e+00, -9.99984064e-01, -6.63199557e-01,
        -9.99951050e-01, -6.60983217e-01, -9.99996016e-01,
         1.46929898e-01],
       [ 0.00000000e+00,  3.99993626e-03,  3.35851709e-01,
         7.01376880e-03,  3.37112194e-01,  1.99600401e-03,
        -9.72560477e-01]])
"""