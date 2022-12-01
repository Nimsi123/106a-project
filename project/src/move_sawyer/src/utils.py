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
  manipulator_jacobian = np.array([[ 8.00000000e-02,  3.16194961e-01,  2.73718737e-01,
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

  new_manipulator_jacobian = np.zeros(manipulator_jacobian.shape)

  # transform xi's to their new orientation
  num_joints = manipulator_jacobian.shape[1]
  for i in range(num_joints):
    g = kfs.prod_exp(manipulator_jacobian[:, :i], thetas[:i])
    Ad_g = SE3_to_Adjoint(g)
    new_manipulator_jacobian[:, i] = Ad_g @ manipulator_jacobian[:, i]

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

def set_spatial_velocity(limb, desired_spatial_velocity, seconds):
  joints = limb.joint_names() # ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

  start = time.time()
  while (time.time() - start) < seconds and not rospy.is_shutdown():
    current_thetas = get_current_joint_angles(limb)
    manip_jacob = get_manip_jacob(current_thetas)

    theta_dot = np.linalg.pinv(manip_jacob) @ desired_spatial_velocity / (2 * np.pi)
    desired_thetas = current_thetas + theta_dot

    # print(current_thetas)
    # print(theta_dot)
    # print(desired_thetas)
    # print("---------")

    for i, theta_delta in enumerate(theta_dot[:7]):

      joint_name = joints[i]
      desired_theta = desired_thetas[i]
      control_joint_to_desired_angle(limb, joint_name, desired_theta)
      

def move_joints_to_zero_config(limb, joints):
	for i in range(len(joints)):
		joint_name = joints[i]
		control_joint_to_desired_angle(limb, joint_name, 0)
		