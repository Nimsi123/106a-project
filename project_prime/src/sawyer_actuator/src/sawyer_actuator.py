import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import MultiDOFJointTrajectory
import intera_interface
import time

def control_joints_to_desired_angles(limb, desired_angles):
  joint_names = limb.joint_names()
  joint_command = dict(zip(joint_names, desired_angles))
  limb.set_joint_position_speed(0.3)

  def close():
    return all([np.isclose(limb.joint_angle(joint_name), desired_angle, atol = 0.01) for joint_name, desired_angle in zip(joint_names, desired_angles)])

  while not close() and not rospy.is_shutdown():
      start = time.time()
      while (time.time() - start) < 4:
        limb.set_joint_positions(joint_command)

def actuator():
  status_pub = rospy.Publisher('actuation_status', String, queue_size=10)
  free = True

  limb = intera_interface.Limb('right')
  limb.set_joint_position_speed(1.0)

  print("Actuator ready.")

  def actuator_helper(desired_thetas):
    nonlocal free

    if free != True:
      print("Oh shit, moving too fast.")
      return

    free = False
    status_pub.publish(f"Actuation started")
    
    desired_thetas = desired_thetas.data
    print(f"received {desired_thetas}")
    control_joints_to_desired_angles(limb, desired_thetas)

    status_pub.publish(f"Actuation completed")
    free = True

  return actuator_helper

if __name__ == '__main__':
    rospy.init_node('sawyer_actuator', anonymous=True)

    callback = actuator()
    rospy.Subscriber("actuation_path", Float64MultiArray, callback)
    rospy.spin()
