#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_commander import MoveGroupCommander
import numpy as np
import warnings
import time
import intera_interface

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

def main():
    warnings.filterwarnings("error")

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    limb = intera_interface.Limb('right')
    limb.set_joint_position_speed(1.0)

    for i in range(points.shape[1]):
        if rospy.is_shutdown():
            break
        point = points[:, i]

        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper_tip"
        request.ik_request.pose_stamped.header.frame_id = "base"     
        
        try:
            response = compute_ik(request)
            group = MoveGroupCommander("right_arm")
            group.set_position_target(point)
            desired_thetas = best_ik_solution(group)
            control_joints_to_desired_angles(limb, desired_thetas)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def test():
    warnings.filterwarnings("error")

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    limb = intera_interface.Limb('right')
    limb.set_joint_position_speed(1.0)

    control_joints_to_desired_angles(limb, [0, 0, 0, 0, 0, 0, 0])

    r = 0.8
    time_steps = np.linspace(-np.pi / 4, np.pi / 4, 5)
    x = r * np.cos(time_steps)
    y = r * np.sin(time_steps)
    z = 0.5 * np.ones(len(time_steps))
    points = np.vstack((x, y, z))

    ik_results = []
    for i in range(points.shape[1]):
        point = points[:, i]
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = "base"  
        try:
            response = compute_ik(request)
            group = MoveGroupCommander("right_arm")
            
            group.set_position_target(point) # Setting just the position without specifying the orientation

            desired_thetas = get_best_plan_angles(group)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        ik_results.append(desired_thetas)

    for desired_thetas in ik_results:
        control_joints_to_desired_angles(limb, desired_thetas)

# Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('service_query')
    test()
