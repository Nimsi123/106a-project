#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.5
        request.ik_request.pose_stamped.pose.position.y = 0.5
        request.ik_request.pose_stamped.pose.position.z = 0.0        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            # print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plans = []
            best_plan = 0
            min_energy = 100000000
            for i in range(0,5):
                plan = group.plan()
                plans.append(plan)
                print(plan[1])
                joint_trajectory_positions = [x.positions for x in plan[1].joint_trajectory.points]
                print(joint_trajectory_positions)
                plan_avg_energy = get_avg_joint_energy(joint_trajectory_positions)
                print(plan_avg_energy)
                if plan_avg_energy < min_energy:
                    min_energy = plan_avg_energy
                    best_plan = plan
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")

            print(min_energy)

            # plan = group.plan()
            # print(plan[1].joint_trajectory.points)
            # joint_trajectory_positions = [x.positions for x in plan[1].joint_trajectory.points]
            # print(len(joint_trajectory_positions))
            # print(joint_trajectory_positions[0])
            
            
            # # Execute IK if safe
            # if user_input == 'y':
            #     group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def get_avg_joint_energy(all_joint_trajectory_positions):
    joint_energys = []
    for i in range(0,7):
        joint_trajectory_positions = [x[i] for x in all_joint_trajectory_positions]
        joint_energy = get_joint_energy(joint_trajectory_positions)
        joint_energys.append(joint_energy)
    average_energy = sum(joint_energys)/len(joint_energys)
    return average_energy


def get_joint_energy(joint_trajectory_positions):
    energy = sum([x**2 for x in joint_trajectory_positions])
    return energy


# Python's syntax for a main() method
if __name__ == '__main__':
    main()
