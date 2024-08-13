#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_arm_and_gripper():
    rospy.init_node('ur5_arm_mover', anonymous=True)

    # Create an action client
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for joint trajectory action server...")
    client.wait_for_server()

    # Define the joint names and target positions
    joint_names = [
        'shoulder_pan_joint',  # Added missing joint
        'shoulder_lift_joint', 
        'elbow_joint', 
        'wrist_1_joint', 
        'wrist_2_joint', 
        'wrist_3_joint', 
        'robotiq_85_left_knuckle_joint'  # Correct joint for gripper based on URDF
    ]
    target_positions = [0.0, -1.5, 1.0, 0.0, 0.0, 0.0, 0.1]  # Values for each joint, including gripper

    # Define the goal (trajectory to be followed by the arm)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = target_positions
    point.time_from_start = rospy.Duration(5.0)  # Move time in seconds
    goal.trajectory.points.append(point)

    # Send the goal
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(6.0))

    # Check the result
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Arm and gripper successfully moved")
    else:
        rospy.loginfo("Failed to move arm and gripper")

if __name__ == '__main__':
    try:
        move_arm_and_gripper()
    except rospy.ROSInterruptException:
        pass
