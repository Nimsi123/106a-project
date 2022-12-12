import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from moveit_commander import MoveGroupCommander
import numpy as np
import warnings

def best_ik_solution(p):
    """
    Returns the ik result that minimizes the largest joint change.
    """

    group = MoveGroupCommander("right_arm")
    group.set_position_target([p.x, p.y, p.z])

    possible_solutions = []
    for i in range(70):
        plan = group.plan()
        joint_trajectory_angles = np.array([list(x.positions) for x in plan[1].joint_trajectory.points])
        
        abs_change_in_theta = np.abs(np.array(joint_trajectory_angles[-1]) - np.array(joint_trajectory_angles[0]))
        possible_solutions.append( (joint_trajectory_angles[-1], np.max(abs_change_in_theta)) )

    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])
    return best_solution

def plan_path(max_publishing_freq):

  path_pub = rospy.Publisher('actuation_path', Float64MultiArray, queue_size=10)
  sleeper = rospy.Rate(max_publishing_freq)

  print("Path planner ready.")

  def plan_path_helper(p):
    print(f"ik result for {p}")   

    desired_thetas = best_ik_solution(p)

    arr = Float64MultiArray()
    arr.data = desired_thetas
    
    print(f"desired thetas {desired_thetas}")
    path_pub.publish(arr)
    sleeper.sleep()

  return plan_path_helper

if __name__ == '__main__':
  rospy.init_node('path_planner', anonymous=True)
  min_publishing_period = 3
  max_publishing_freq = 1 / min_publishing_period

  callback = plan_path(max_publishing_freq)
  rospy.Subscriber("next_sawyer_loc", Point, callback)
  rospy.spin()
