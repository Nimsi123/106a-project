import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import MultiDOFJointTrajectory

def control_joints_to_desired_angles(limb, desired_angles):
  joint_names = limb.joint_names()
  joint_command = dict(zip(joint_names, desired_angles))
  limb.set_joint_position_speed(0.3)

  def close():
    return all([np.isclose(limb.joint_angle(joint_name), desired_angle, atol = 0.01) for joint_name, desired_angle in zip(joint_names, desired_angles)])

  while not close() and not rospy.is_shutdown():
      start = time.time()
      while (time.time() - start) < 5:
        limb.set_joint_positions(joint_command)

def actuator(min_publishing_period):
  def actuator_helper(trajectory):
    status_pub = rospy.Publisher('actuation_status', String, queue_size=10)
    
    # actuate the path


    r = rospy.Rate(min_publishing_period)
    r.sleep()
    status_pub.publish(f"Actuation completed. {trajectory}")
  return actuator_helper

if __name__ == '__main__':
    rospy.init_node('sawyer_actuator', anonymous=True)
    min_publishing_period = 3
    
    rospy.Subscriber("actuation_path", String, actuator(min_publishing_period)) # replace String with MultiDOFJointTrajectory
    rospy.spin()
