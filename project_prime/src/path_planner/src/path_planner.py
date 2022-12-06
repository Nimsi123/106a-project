import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import MultiDOFJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_commander import MoveGroupCommander
import numpy as np
import warnings
import time
import intera_interface

def best_ik_solution(group):
    """
    Returns the ik result that minimizes the largest joint change.
    """
    possible_solutions = []
    for i in range(40):
        plan = group.plan()
        joint_trajectory_angles = np.array([list(x.positions) for x in plan[1].joint_trajectory.points])
        
        abs_change_in_theta = np.abs(np.array(joint_trajectory_angles[-1]) - np.array(joint_trajectory_angles[0]))
        possible_solutions.append( (joint_trajectory_angles[-1], np.max(abs_change_in_theta)) )

    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])
    return best_solution

def plan_path(max_publishing_freq):

  limb = intera_interface.Limb('right')
  limb.set_joint_position_speed(1.0)

  def plan_path_helper(p):
    path_pub = rospy.Publisher('actuation_path', Float64MultiArray, queue_size=10) # replace String with MultiDOFJointTrajectory

    

    # plan the path
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper_tip"
    request.ik_request.pose_stamped.header.frame_id = "base"     
    
    try:
        response = compute_ik(request)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    group = MoveGroupCommander("right_arm")
    group.set_position_target([p.x, p.y, p.z])
    desired_thetas = best_ik_solution(group)
    arr = Float64MultiArray()
    arr.data = desired_thetas

    r = rospy.Rate(max_publishing_freq)
    r.sleep()
    print("publishing", arr)
    path_pub.publish(arr)
  return plan_path_helper

if __name__ == '__main__':
  rospy.init_node('path_planner', anonymous=True)
  min_publishing_period = 3
  max_publishing_freq = 1 / min_publishing_period

  print("waiting for service")
  rospy.wait_for_service('compute_ik')
  print("service ready")
  compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

  rospy.Subscriber("next_sawyer_loc", Point, plan_path(max_publishing_freq))
  rospy.spin()
