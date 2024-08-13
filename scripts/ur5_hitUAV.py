#!/usr/bin/env python3

import rospy
from trac_ik_python.trac_ik import IK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Vector3, Quaternion

def move_through_cartesian_points():
    rospy.init_node('ur5_move_through_points')

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

    # Define the Cartesian trajectory points
    pose_list = [
        Pose(
            Vector3(-0.109, -0.50, 0.142), Quaternion(0.4997, -0.5002, 0.4999, 0.5002)
        ),
        Pose(
            Vector3(-0.109, -0.80, 0.142), Quaternion(0.4997, -0.5002, 0.4999, 0.5002)
        ),
        Pose(
            Vector3(-0.109, -0.60, 0.142), Quaternion(0.4997, -0.5002, 0.4999, 0.5002)
        ),
    ]

    for i, pose in enumerate(pose_list):
        # Extract position and orientation from each point
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # Compute joint angles for the given Cartesian point
        joint_angles = ik_solver.get_ik([0]*6, x, y, z, qx, qy, qz, qw)

        if joint_angles:
            # Define the trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_angles
            point.time_from_start = rospy.Duration((i+1) * 5.0)  # Time to reach the goal

            # Add the point to the trajectory
            joint_trajectory.points.append(point)
        else:
            rospy.logerr(f"IK solution not found for point {i+1}")

    if joint_trajectory.points:
        # Publish the joint trajectory
        rospy.loginfo("Publishing joint trajectory...")
        joint_command_pub.publish(joint_trajectory)
        rospy.loginfo("Joint trajectory published.")
    else:
        rospy.logerr("No valid IK solutions were found for the provided points.")

    # Keep the node alive to ensure the trajectory is executed
    rospy.sleep(len(pose_list) * 6.0)  # Ensure there's enough time for execution

if __name__ == '__main__':
    try:
        move_through_cartesian_points()
    except rospy.ROSInterruptException:
        pass
