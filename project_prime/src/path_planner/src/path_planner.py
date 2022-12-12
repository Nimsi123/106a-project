import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from moveit_commander import MoveGroupCommander
import numpy as np
import warnings
import time
import intera_interface

from geometry_msgs.msg import Point

def test_timing():
    arc = [
        np.array((0.61, 0.64, 0.22)),
        np.array((0.67, 0.50, 0.47)),
        np.array((0.69, 0.31, 0.67)),
        np.array((0.68, 0.06, 0.70)),
        np.array((0.61, -0.25, 0.67)),
        np.array((0.56, -0.44, 0.55)),
        np.array((0.56, -0.48, 0.42)),
        np.array((0.63, -0.43, 0.39))
    ]

    start = time.time()
    counter = 0
    for point in arc:
        p = Point()
        p.x, p.y, p.z = point
        for _ in range(10):
            best_ik_solution(p)
            counter += 1
    end = time.time()

    print(f"Average planning time: {(end - start) / counter}")


def best_ik_solution(p):
    """
    Returns the ik result that minimizes the largest joint change.
    """

    group = MoveGroupCommander("right_arm")
    group.set_position_target([p.x, p.y, p.z])

    possible_solutions = []
    for i in range(50):
        plan = group.plan()
        joint_trajectory_angles = np.array([list(x.positions) for x in plan[1].joint_trajectory.points])
        
        abs_change_in_theta = np.abs(np.array(joint_trajectory_angles[-1]) - np.array(joint_trajectory_angles[0]))
        possible_solutions.append( (joint_trajectory_angles[-1], np.max(abs_change_in_theta)) )

    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])
    return best_solution

def best_ik_solution2(p):
    """
    Returns the ik result that minimizes the largest joint change.
    """
    from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    def one_ik_sol():
        
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        link = "right_gripper_tip"
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = "base"
        request.ik_request.pose_stamped.pose.position.x = p.x
        request.ik_request.pose_stamped.pose.position.y = p.y
        request.ik_request.pose_stamped.pose.position.z = p.z
        response = compute_ik(request)

        d = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        if not d:
            return -1

        # print(d)
        solution = np.array([d[f"right_j{i}"] for i in range(7)])

        return solution

    current_thetas = get_current_joint_angles()

    start = time.time()
    possible_solutions = []
    for i in range(50):
        desired_thetas = one_ik_sol()
        if type(desired_thetas) == int:
            continue
        abs_change_in_theta = np.abs(desired_thetas - current_thetas)
        possible_solutions.append( (desired_thetas, np.max(abs_change_in_theta)) )

    temp = [x[1] for x in possible_solutions]
    print(np.mean(temp), np.std(temp), len(np.unique(temp)))
    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])
    end = time.time()

    print(best_cost)
    if best_cost > 2.5:
        return best_ik_solution(p)
    return best_solution

def plan_path(max_publishing_freq):

  path_pub = rospy.Publisher('actuation_path', Float64MultiArray, queue_size=10)
  sleeper = rospy.Rate(max_publishing_freq)

  print("Path planner ready.")

  def plan_path_helper(p):
    # print(f"ik result for {p}")   

    start = time.time()
    desired_thetas = best_ik_solution2(p)

    arr = Float64MultiArray()
    arr.data = desired_thetas
    
    # print(f"desired thetas {desired_thetas}")
    path_pub.publish(arr)
    print(f"Planning took {time.time() - start} seconds.")
    sleeper.sleep()

  return plan_path_helper

def get_current_joint_angles():
  limb = intera_interface.Limb('right')
  current_angles = []
  for name in limb.joint_names():
    current_angles.append(limb.joint_angle(name))

  return np.array(current_angles)

if __name__ == '__main__':
  rospy.init_node('path_planner', anonymous=True)
  min_publishing_period = 0.5
  max_publishing_freq = 1 / min_publishing_period

  # test_timing()

  # p = Point()
  # p.x, p.y, p.z = [0.61, 0.64, 0.22]
  # for _ in range(5):
  #   best_ik_solution2(p)

  # get_current_joint_angles()

  callback = plan_path(max_publishing_freq)
  rospy.Subscriber("next_sawyer_loc", Point, callback)
  rospy.spin()
