#!/usr/bin/env python3

import rospy
from trac_ik_python.trac_ik import IK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Vector3, Quaternion

def binary_search_limit(axis, low, high, fixed_values, ik_solver, epsilon=0.01):
    """Binary search to find the robot's limit along one axis."""
    while high - low > epsilon:
        mid = (high + low) / 2.0
        pos = list(fixed_values)  # Copy the fixed_values list
        pos[axis] = mid  # Set the axis we're testing

        # Try to find a valid IK solution
        joint_angles = ik_solver.get_ik([0]*6, pos[0], pos[1], pos[2], 0, 0, 0, 1)
        
        if joint_angles:
            low = mid  # Move up the lower bound
        else:
            high = mid  # Move down the upper bound
    
    return low

def find_robot_limits():
    rospy.init_node('ur5_find_limits')

    # Initialize the IK solver
    ik_solver = IK("base_link", "tool0")

    # Start with some realistic fixed values for the non-tested axes
    fixed_values_x = [0.0, 0.0, 0.5]  # Fix Y and Z when testing X-axis
    fixed_values_y = [0.0, 0.0, 0.5]  # Fix X and Z when testing Y-axis
    fixed_values_z = [0.5, 0.0, 0.0]  # Fix X and Y when testing Z-axis

    # Define the limits we are trying to find
    x_limit = binary_search_limit(0, 0.0, 1.2, fixed_values_x, ik_solver)
    x_neg_limit = -binary_search_limit(0, 0.0, 1.2, fixed_values_x, ik_solver)

    y_limit = binary_search_limit(1, 0.0, 1.2, fixed_values_y, ik_solver)
    y_neg_limit = -binary_search_limit(1, 0.0, 1.2, fixed_values_y, ik_solver)

    z_limit = binary_search_limit(2, 0.0, 1.5, fixed_values_z, ik_solver)
    z_neg_limit = -binary_search_limit(2, 0.0, 0.5, fixed_values_z, ik_solver)

    # Print out the found limits
    rospy.loginfo(f"X-axis limit: {x_limit}, -X-axis limit: {x_neg_limit}")
    rospy.loginfo(f"Y-axis limit: {y_limit}, -Y-axis limit: {y_neg_limit}")
    rospy.loginfo(f"Z-axis limit: {z_limit}, Z-min limit: {z_neg_limit}")

if __name__ == '__main__':
    try:
        find_robot_limits()
    except rospy.ROSInterruptException:
        pass
