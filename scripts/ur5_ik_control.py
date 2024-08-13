#!/usr/bin/env python3

import rospy
from trac_ik_python.trac_ik import IK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Vector3, Quaternion
from tf.transformations import quaternion_from_euler

def move_to_first_cartesian_point():
    rospy.init_node('ur5_move_to_first_point')

    # Initialize the IK solver
    ik_solver = IK("base_link", "tool0")

    # Publisher for the joint trajectory
    joint_command_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1.0)  # Give time to the publisher to connect

    # Prepare the joint trajectory message
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    ]

    # Extract the first Cartesian point
    first_point = Pose(
        Vector3(-0.109, -0.50, 0.142), Quaternion(0.4997, -0.5002, 0.4999, 0.5002)
    )

    # Extract position and orientation from the first point
    x = first_point.position.x
    y = first_point.position.y
    z = first_point.position.z

    # Quaternion components directly used in IK solver
    qx = first_point.orientation.x
    qy = first_point.orientation.y
    qz = first_point.orientation.z
    qw = first_point.orientation.w

    # Compute joint angles for the given Cartesian point
    joint_angles = ik_solver.get_ik([0]*6, x, y, z, qx, qy, qz, qw)

    if joint_angles:
        # Define the trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(5.0)  # Time to reach the goal

        # Add the point to the trajectory
        joint_trajectory.points.append(point)

        # Publish the joint trajectory
        rospy.loginfo("Publishing joint trajectory...")
        joint_command_pub.publish(joint_trajectory)
        rospy.loginfo("Joint trajectory published.")
    else:
        rospy.logerr("IK solution not found for the first Cartesian point.")

    # Keep the node alive to ensure the trajectory is executed
    rospy.sleep(6.0)

if __name__ == '__main__':
    try:
        move_to_first_cartesian_point()
    except rospy.ROSInterruptException:
        pass
