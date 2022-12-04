import numpy as np
import kin_func_skeleton as kfs
import rospy
import time

def SE3_to_Adjoint(g):
  R = g[:3, :3]
  p = g[:3, 3]
  
  return np.vstack((
    np.hstack((R, kfs.skew_3d(p) @ R)),
    np.hstack((np.zeros((3, 3)), R))
    ))

def get_manip_jacob(thetas):
  manipulator_jacobian = np.array([[-0.00000000e+00, -3.16242120e-01,  1.16080254e-03,
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

  new_manipulator_jacobian = np.zeros(manipulator_jacobian.shape)

  # transform xi's to their new orientation
  num_joints = manipulator_jacobian.shape[1]
  for i in range(num_joints):
    g = kfs.prod_exp(manipulator_jacobian[:, :i], thetas[:i])
    Ad_g = SE3_to_Adjoint(g)
    new_manipulator_jacobian[:, i] = Ad_g @ manipulator_jacobian[:, i]

  # manipulator_jacobian = new_manipulator_jacobian

  return new_manipulator_jacobian

def get_current_joint_angles(limb):
  current_angles = []
  for name in limb.joint_names():
    current_angles.append(limb.joint_angle(name))

  return np.array(current_angles)

def control_joint_to_desired_angle(limb, joint_name, desired_angle):
  # print(f"Moving joint {joint_name} to {desired_angle}.")
  joint_command = {joint_name: desired_angle}
  limb.set_joint_position_speed(0.3)

  while not np.isclose(limb.joint_angle(joint_name), desired_angle, atol = 0.01) and not rospy.is_shutdown():
    # print(limb.joint_angle(joint_name))
    limb.set_joint_positions(joint_command)

def control_joints_to_desired_angles(limb, joint_names, desired_angles):

  joint_command = dict(zip(joint_names, desired_angles))
  limb.set_joint_position_speed(0.3)

  def close():
    return all([np.isclose(limb.joint_angle(joint_name), desired_angle, atol = 0.01) for joint_name, desired_angle in zip(joint_names, desired_angles)])

  while not close() and not rospy.is_shutdown():
    limb.set_joint_positions(joint_command)

def set_spatial_velocity(limb, desired_spatial_velocity, seconds):
  joint_names = limb.joint_names() # ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

  start = time.time()
  while (time.time() - start) < seconds and not rospy.is_shutdown():
    current_thetas = get_current_joint_angles(limb)
    manip_jacob = get_manip_jacob(current_thetas)

    theta_dot = np.linalg.pinv(manip_jacob) @ desired_spatial_velocity / (2 * np.pi)
    desired_thetas = current_thetas + theta_dot

    # print("Current thetas: ", current_thetas)
    # print("Theta dot: ", theta_dot)
    # print("(Normalized) Theta dot: ", theta_dot / np.linalg.norm(theta_dot))
    # print(desired_thetas)
    # print("---------")

    control_joints_to_desired_angles(limb, joint_names, desired_thetas)

    # for i, theta_delta in enumerate(theta_dot):

    #   joint_name = joints[i]
    #   desired_theta = desired_thetas[i]
    #   control_joint_to_desired_angle(limb, joint_name, desired_theta)
      

def move_joints_to_zero_config(limb, joints):
	for i in range(len(joints)):
		joint_name = joints[i]
		control_joint_to_desired_angle(limb, joint_name, 0)
		