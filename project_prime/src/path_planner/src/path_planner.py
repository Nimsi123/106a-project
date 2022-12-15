import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import numpy as np
import warnings
import time
import intera_interface

def test_timing(best_ik_solution):
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

def get_current_joint_angles(limb):
    return np.array([limb.joint_angle(name) for name in limb.joint_names()])

def one_ik_sol_moveit(p):
    group = MoveGroupCommander("right_arm")
    group.set_position_target([p.x, p.y, p.z])

    def helper():
        plan = group.plan()
        return np.array(plan[1].joint_trajectory.points[-1].positions), True
        
    return helper

def one_ik_sol_compute_ik(p):
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    link = "right_gripper_tip"
    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = "base"
    request.ik_request.pose_stamped.pose.position.x = p.x
    request.ik_request.pose_stamped.pose.position.y = p.y
    request.ik_request.pose_stamped.pose.position.z = p.z

    def helper():
        response = compute_ik(request)
        joint_state = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        if not joint_state:
            return None, False
        final_thetas = np.array([joint_state[f"right_j{i}"] for i in range(7)])
        return final_thetas, True

    return helper

def ik_sol_cost(initial_thetas, final_thetas):
    return np.max(np.abs(final_thetas - initial_thetas))

def best_solution(one_sol, cost, attempts, alternate_sol = None, end_early = False):
    start = time.time()

    possible_solutions = []
    for i in range(attempts):
        sol, valid_sol = one_sol()
        if valid_sol == False:
            continue
        
        sol_cost = cost(sol)
        if end_early and sol_cost < 2.5:
            print(f"Quitting early after {i} steps with cost {sol_cost}.")
            return sol, sol_cost
        possible_solutions.append( (sol, sol_cost) )

    if len(possible_solutions) == 0:
        return None, False
    (best_solution, best_cost) = min(possible_solutions, key = lambda x: x[1])

    end = time.time()
    print(f"Best cost out of {len(possible_solutions)} attempts: {best_cost}.")
    # print([round(x[1], 2) for x in possible_solutions])
    return best_solution, best_cost

def plan_path(max_publishing_freq):

    limb = intera_interface.Limb('right')
    initial_thetas = get_current_joint_angles(limb)
    path_pub = rospy.Publisher('actuation_path', Float64MultiArray, queue_size=10)
    sleeper = rospy.Rate(max_publishing_freq)

    print("Path planner ready.")

    def plan_path_helper(p):
        nonlocal initial_thetas

        start = time.time()

        cost = lambda final_thetas: ik_sol_cost(initial_thetas, final_thetas)
        one_ik_sol = one_ik_sol_compute_ik(p)
        best_thetas, best_cost = best_solution(one_ik_sol, cost, attempts = 50)

        if best_cost == False or best_cost > 2.5:
            one_ik_sol = one_ik_sol_moveit(p)
            thetas, c = best_solution(one_ik_sol, cost, attempts = 50, end_early = True)
            if best_cost == False or c < best_cost:
                best_thetas, best_cost = thetas, c

        arr = Float64MultiArray()
        arr.data = best_thetas
        initial_thetas = best_thetas # set starting point for the next planner call
        path_pub.publish(arr)

        print(f"Planning took {time.time() - start} seconds.")
        # sleeper.sleep()

    return plan_path_helper

if __name__ == '__main__':
    rospy.init_node('path_planner', anonymous=True)
    min_publishing_period = 0.5
    max_publishing_freq = 1 / min_publishing_period

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    callback = plan_path(max_publishing_freq)
    rospy.Subscriber("next_sawyer_loc", Point, callback)
    rospy.spin()
