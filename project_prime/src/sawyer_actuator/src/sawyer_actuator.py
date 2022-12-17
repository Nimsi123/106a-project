import rospy
import numpy as np
import intera_interface
import time
from std_msgs.msg import Float64MultiArray

CHECK_TIME_AT_DESIRED_ANGLES = 0.1

def control_joints_to_desired_angles(limb, desired_angles):
  joint_names = limb.joint_names()
  joint_command = dict(zip(joint_names, desired_angles))

  def close():
    return all([np.isclose(limb.joint_angle(joint_name), desired_angle, atol = 0.01) for joint_name, desired_angle in zip(joint_names, desired_angles)])

  while not close() and not rospy.is_shutdown():
      start = time.time()
      while (time.time() - start) < CHECK_TIME_AT_DESIRED_ANGLES:
        limb.set_joint_positions(joint_command)

def actuator():
  free = True

  limb = intera_interface.Limb('right')
  limb.set_joint_position_speed(0.8)
  print("Actuator ready.")

  def actuator_helper(desired_thetas):
    nonlocal free

    if free != True:
      print("Oh shit, moving too fast.")
      return

    free = False
    start = time.time()
    control_joints_to_desired_angles(limb, desired_thetas.data)
    print(f"Actuation took {time.time() - start} seconds.")
    free = True

  return actuator_helper

if __name__ == '__main__':
    rospy.init_node('sawyer_actuator', anonymous=True)

    callback = actuator()
    rospy.Subscriber("actuation_path", Float64MultiArray, callback)
    rospy.spin()
