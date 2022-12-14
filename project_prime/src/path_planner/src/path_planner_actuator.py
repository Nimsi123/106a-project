import rospy
from moveit_commander import MoveGroupCommander
import numpy as np
import intera_interface
import time
from geometry_msgs.msg import Point

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

def best_ik_solution(p):
    """
    Returns the ik result that minimizes the largest joint change.
    """

    group = MoveGroupCommander("right_arm")
    group.set_position_target([p.x, p.y, p.z])

    possible_solutions = []
    for i in range(80):
        plan = group.plan()
        joint_trajectory_angles = np.array([list(x.positions) for x in plan[1].joint_trajectory.points])
        
        abs_change_in_theta = np.abs(np.array(joint_trajectory_angles[-1]) - np.array(joint_trajectory_angles[0]))
        possible_solutions.append( (joint_trajectory_angles[-1], np.max(abs_change_in_theta)) )

    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])
    return best_solution

def path_planner_and_actuator():

  limb = intera_interface.Limb('right')
  limb.set_joint_position_speed(1.0)
  print("Actuator ready to run.")

  def helper(p):
    print(f"Working on {p}")
    desired_thetas = best_ik_solution(p)
    
    # print(f"desired thetas {desired_thetas}")
    control_joints_to_desired_angles(limb, desired_thetas)

  return helper

if __name__ == '__main__':
  rospy.init_node('path_planner', anonymous=True)

  callback = path_planner_and_actuator()
  rospy.Subscriber("next_sawyer_loc", Point, callback)
  rospy.spin()