#!/usr/bin/env python

'''
maintenance:

ERC Remote 2021 Maintenance Task Demo; gets target button and state from operator, then performs marker based operation semi-autonomously

'''

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
# from math import pi
import math 
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# roslaunch panel_description gazebo.launch yaw:=0.3 x:=-0.2

move_group = moveit_commander.MoveGroupCommander
display_trajectory_publisher = rospy.Publisher
oldPose = Pose()
pose = Pose()
pose_goal = Pose()


def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])



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
    rospy.init_node('maintenance', anonymous=True)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
    go_named_target("panel_up")
    rospy.sleep(3)

    rospy.Subscriber("/aruco_single/pose", PoseStamped, callback)
    rospy.spin()

def button_pose(btn_index, state):
    global pose
    # print btn_index, state
    pose_goal = Pose()
    pose_goal.position.x = pose.position.x
    pose_goal.position.y = pose.position.y
    pose_goal.position.z = pose.position.z

    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    # print"euler: ",euler
    pose_goal.orientation = Quaternion(*quaternion_from_euler(euler[0] - 4.71, euler[1], euler[2] + 1.57))
    # pose_goal.orientation = Quaternion(*quaternion_from_euler(-3.14, 0, 1.57))


    pose_goal.position.x -= 0.025
    pose_goal.position.y -= 0.141
    pose_goal.position.z -= 0.055

    if(btn_index == 0 or btn_index == 4):
        pose_goal.position.x -= 0.02909
        # print("1. Column")
    elif(btn_index == 2 or btn_index == 6):
        pose_goal.position.x += 0.05091
        # print("3. Column")
    elif(btn_index == 3 or btn_index == 7):
        pose_goal.position.x += 0.08
        # print("4. Column")
    # else:
        # print("2. Column")

    
    if(btn_index > 3):
        # print("Down")
        pose_goal.position.z -= 0.16
    # else:
        # print("Up")

    if(state == 0):
        # print("OFF")
        pose_goal.position.z -= 0.035
    # else:
        # print("ON")

    # pre-approach WILL BE IMPLEMENTED
    # if(pre == 1):
    #     print("Pre-approach")
    #     pose_goal.position.y - 0.02
    # else:
    #     print("Tap")

    # rotations WILL BE IMPLEMENTED
    '''
    axis = [pose.position.x, pose.position.y, pose.position.z]
    vector = [pose_goal.position.x, pose_goal.position.y, pose_goal.position.z]
    theta = euler[2] + 1.57
    result = np.dot(rotation_matrix(axis, theta), vector)
    pose_goal.position.x = result[0]
    pose_goal.position.y = result[1]
    pose_goal.position.z = result[2]
    print "pose_goal: ",np.dot(rotation_matrix(axis, theta), vector)
    '''

    return pose_goal


def go_to_pose_goal(pose_goal):
    global move_group
    global pose
    
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
'''
    # pose operations moved button_pose method
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
    pose_goal.position.x = pose.position.x - 0.025

    #buttons at 0,3078 = y -0.14
    pose_goal.position.y = pose.position.y - 0.141
    pose_goal.position.z = pose.position.z - 0.055
    # pose_goal.position.z = pose.position.z + 0.05
'''

def go_named_target(target):
    global move_group
    move_group.set_named_target(target)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    # print"going to ", target, " position..."

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
        print("### Marker Detected ###")
        print"Position:"
        print"x: %.4f" % marker_position.x, ", y: %.4f" % marker_position.y, ", z: %.4f" % marker_position.z

        print"Orientation:"
        print"x: %.4f" % marker_orientation.x, ", y: %.4f" % marker_orientation.y, ", z: %.4f" % marker_orientation.z, ", w: %.4f" % marker_orientation.w
        print""
        print""

        #get input and handle on off
        num = ""
        named_target = "panel_up"
        while(True):
            num, state = raw_input("Enter button num and state: ").split()
            if(state == 'ON'):
                state = 1
            elif(state == 'OFF'):
                state = 0
            num = int(num)
            # num = int(input("Enter button num "))
            # state = int(input("and state: "))

            if(num < 4):
                named_target = "panel_up"
            else:
                named_target = "panel_down"

            try:
                go_named_target(named_target)
                rospy.sleep(0.25)
                go_to_pose_goal(button_pose(num, state))
                rospy.sleep(0.25)
                go_named_target(named_target)
                print("OK")
            except:
                print("Emergency stop")

'''
        go_to_pose_goal(button_pose(0, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(0, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_up")
        rospy.sleep(1)

        go_to_pose_goal(button_pose(0, 0, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(0, 0, 0))
        rospy.sleep(1)
        go_named_target("panel_up")
        rospy.sleep(1)
        
        go_to_pose_goal(button_pose(1, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(1, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_up")
        rospy.sleep(1)
        
        go_to_pose_goal(button_pose(2, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(2, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_up")
        rospy.sleep(1)
        
        go_to_pose_goal(button_pose(3, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(3, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_up")
        rospy.sleep(1)
        
        go_named_target("panel_down")
        rospy.sleep(1)
        go_to_pose_goal(button_pose(4, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(4, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_down")
        rospy.sleep(1)
        
        go_to_pose_goal(button_pose(5, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(5, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_down")
        rospy.sleep(1)
        
        go_to_pose_goal(button_pose(6, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(6, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_down")
        rospy.sleep(1)
        
        go_to_pose_goal(button_pose(7, 1, 1))
        rospy.sleep(0.5)
        go_to_pose_goal(button_pose(7, 1, 0))
        rospy.sleep(1)
        go_named_target("panel_down")
        rospy.sleep(1)
'''      





def main():
    init_tracker()


if __name__ == '__main__':
    # init_tracker()
    main()


# roslaunch panel_description gazebo.launch x:=0.075 y:=1
# roslaunch panel_description gazebo.launch x:=0.1 y:=1
