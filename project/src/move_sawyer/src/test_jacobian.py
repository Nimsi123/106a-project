import numpy as np

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION
import utils

desired_spatial_velocities = [
	{
		"description": "",
		"xi": [3, 0, 0, 0, 0, 0],
	}, 

]
"""
{
	"description": "Rotate the third joint",
	"xi": manipulator_jacobian[:, 2],
}, 
{
	"description": "Move the arm backwards",
	"xi": [-1, 0, 0, 0, 0, 0],
}, 
{
	"description": "Move the arm to the right",
	"xi": [0, 1, 0, 0, 0, 0],
}, 
{
	"description": "Move the arm straight up",
	"xi": [0, 0, 1, 0, 0, 0],
}, 
"""

rp = intera_interface.RobotParams()
valid_limbs = rp.get_limb_names()
if not valid_limbs:
    rp.log_message(("Cannot detect any limb parameters on this robot. "
                    "Exiting."), "ERROR")
    import sys
    sys.exit()

print("Initializing node... ")
rospy.init_node("test_jacobian")
print("Getting robot state... ")
rs = intera_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled

limb = intera_interface.Limb(valid_limbs[0])
joints = limb.joint_names()

def clean_shutdown():
    print("\nExiting example.")
rospy.on_shutdown(clean_shutdown)

rospy.loginfo("Enabling robot...")
rs.enable()

# utils.move_joints_to_zero_config(limb, joints)

# utils.control_joint_to_desired_angle(limb, joints[1], -np.pi / 8)

# try desired trajectories
for spatial_velocity in desired_spatial_velocities:	

	print(spatial_velocity["description"])
	print(spatial_velocity["xi"])

	print("Start?")
	input()

	utils.set_spatial_velocity(limb, spatial_velocity["xi"], 3)

#----------------------
init_manip_jacob = utils.get_manip_jacob(np.array([0, 0, 0, 0, 0, 0, 0]))
# print("Going to start!")
# for i in list(range(init_manip_jacob.shape[1])):
# 	print(f"Move joint {i}?")
# 	input()

# 	utils.move_joints_to_zero_config(limb, joints)
# 	xi = init_manip_jacob[:, i]
# 	utils.set_spatial_velocity(limb, xi, 3)
#----------------------

print("Done.")