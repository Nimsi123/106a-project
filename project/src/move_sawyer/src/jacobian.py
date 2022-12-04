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
# for joint in revolute_joints:
# 	# arrange values to [q.w, q.x, q.y, q.z]
# 	q_z = joint["orientation"][3]
# 	joint["orientation"][1:3] = joint["orientation"][0:2]
# 	joint["orientation"][0] = q_z


manipulator_jacobian = np.zeros((6, 7))
for i, joint in enumerate(revolute_joints):
  q = joint["position"]
  R_SE3 = quaternion_matrix(joint["orientation"])
  w = R_SE3[:3, 2] # z axis is the axis of rotation
  v = - np.cross(w, q)
  xi = np.concatenate((v, w))
  # print(f"Xi {i}: {xi}")
  manipulator_jacobian[:, i] = xi

# print("Manipulator Jacobian")
# print(repr(manipulator_jacobian))

print(manipulator_jacobian[:, 1])

"""
np.array([[-0.00000000e+00, -3.16242120e-01,  1.16080254e-03,
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
"""

