#!/usr/bin/env python

'''
aruco_tracker:

Gets pose from '/aruco_single/pose' and tracks marker with ur3 arm

'''

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion



move_group = moveit_commander.MoveGroupCommander
display_trajectory_publisher = rospy.Publisher
oldPose = Pose()
pose = Pose()
pose_goal = Pose()


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


def init_tracker():
    global moveit_commander
    global move_group
    global display_trajectory_publisher

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('aruco_tracker', anonymous=True)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    rospy.Subscriber("/aruco_single/pose", PoseStamped, callback)
    rospy.spin()


def go_to_pose_goal(x, y, z, qx, qy, qz, qw):
    global move_group
    global pose

    pose_goal = Pose()
    pose_goal.orientation.x = pose.orientation.x
    pose_goal.orientation.y = pose.orientation.y
    pose_goal.orientation.z = pose.orientation.z
    pose_goal.orientation.w = pose.orientation.w

    goal_quaternion = pose.orientation
    # goal_euler = euler_from_quaternion(goal_quaternion)
    # goal_euler.ay -= 3.14
    # pose_goal.orientation = Quaternion(*quaternion_from_euler(goal_euler.ax, goal_euler.ay, goal_euler.az))

    pose_goal.orientation = Quaternion(*quaternion_from_euler(-3.14, 0, 1.57))
    # pose_goal.orientation = goal_quaternion
    pose_goal.position.x = pose.position.x
    pose_goal.position.y = pose.position.y - 0.02
    pose_goal.position.z = pose.position.z + 0.015
    # pose_goal.position.z = pose.position.z + 0.05
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def poseChanged():
    poselist = [pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    oldposelist = [oldPose.position.x, oldPose.position.y, oldPose.position.z,
                oldPose.orientation.x, oldPose.orientation.y, oldPose.orientation.z, oldPose.orientation.w]

    i = 0
    while i < 7:
        if(abs(oldposelist[i] - poselist[i]) > 0.05):
            print""
            print"poseChanged"
            print""
            return True
        i += 1
    return False


def callback(data):
    global oldPose
    global pose
    oldPose = pose
    pose = data.pose

    if(poseChanged()):
        marker_position = pose.position
        marker_orientation = pose.orientation
        print"Position:"
        print"x: %.4f" % marker_position.x, ", y: %.4f" % marker_position.y, ", z: %.4f" % marker_position.z

        print"Orientation:"
        print"x: %.4f" % marker_orientation.x, ", y: %.4f" % marker_orientation.y, ", z: %.4f" % marker_orientation.z, ", w: %.4f" % marker_orientation.w
        print""
        print""

        # rospy.sleep(1)
        go_to_pose_goal(marker_position.x, marker_position.y, marker_position.z, marker_orientation.x, marker_orientation.y, marker_orientation.z, marker_orientation.w)


if __name__ == '__main__':
    init_tracker()
