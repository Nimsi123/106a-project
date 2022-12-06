#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from sawyer_forward_kinematics import get_position_from_fwd_kinematics
import warnings
import time

import intera_interface

def get_current_joint_angles(limb):
  current_angles = []
  for name in limb.joint_names():
    current_angles.append(limb.joint_angle(name))

  return np.array(current_angles)

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


  print("Done actuating Sawyer!")

def main():
    warnings.filterwarnings("error")

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    limb = intera_interface.Limb('right')
    limb.set_joint_position_speed(1.0)

    r = 0.8
    time_steps = np.linspace(-np.pi / 2, np.pi / 2, 5)
    x = r * np.cos(time_steps)
    y = r * np.sin(time_steps)
    z = 0.5 * np.ones(len(time_steps))
    points = np.vstack((x, y, z))

    for i in range(points.shape[1]):
        if rospy.is_shutdown():
            break
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
        
def get_best_plan_angles(group):
    # TODO: penalize solutions that don't take us to exact x, y, z position
    # plans = []
    possible_solutions = []
    for i in range(40):
        plan = group.plan()
        joint_trajectory_angles = np.array([list(x.positions) for x in plan[1].joint_trajectory.points])
        
        change_in_theta = np.abs(np.array(joint_trajectory_angles[-1]) - np.array(joint_trajectory_angles[0]))
        possible_solutions.append( (joint_trajectory_angles[-1], np.max(change_in_theta)) )

    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])
    return best_solution

# def compute_path_distance(positions):
#     total_distance = 0
#     for i in range(1,len(positions)):
#         start_position = positions[i-1]
#         end_position = positions[i]
#         total_distance += np.linalg.norm(end_position - start_position)
#     return total_distance

# def get_best_plan_displacement(group):
#     plans = []
#     best_plan = None
#     min_dist = float('inf')
#     end_angles = []
#     for i in range(50):
#         try:
#             plan = group.plan()
#         # checkpoint = input("check and enter to continue")
#         except Exception:
#             print("No path Found")
#             break


#         plans.append(plan)
#         joint_trajectory_angles = np.array([list(x.positions) for x in plan[1].joint_trajectory.points])

#         positions = np.array([get_position_from_fwd_kinematics(angle) for angle in joint_trajectory_angles])

#         distance = compute_path_distance(positions)
#         print(distance)

#         end_angles.append(joint_trajectory_angles[len(joint_trajectory_angles)-1])

#         if distance < min_dist:
#             min_dist = distance
#             best_plan = plan
#     # print(end_angles)
#     for angles in end_angles:
#         print(angles)
#     print("best plan distance: ", min_dist)
#     # attempting to speed up best plan
#     return best_plan

# def get_avg_joint_energy(all_joint_trajectory_positions):
#     joint_energys = []
#     for i in range(0,7):
#         joint_trajectory_positions = [x[i] for x in all_joint_trajectory_positions]
#         joint_energy = get_joint_energy(joint_trajectory_positions)
#         joint_energys.append(joint_energy)
#     average_energy = sum(joint_energys)/len(joint_energys)
#     return average_energy


# def get_joint_energy(joint_trajectory_positions):
#     energy = sum([x**2 for x in joint_trajectory_positions])
#     return energy


# Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('service_query')
    test()
