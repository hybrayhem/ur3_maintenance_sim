#!/usr/bin/env python

'''
aruco_subscriber:

Subscribes '/aruco_single/pose' and prints on changes to track it

'''

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


_oldPose = Pose()
_pose = Pose()


def poseChanged():
    tolerance = 0.005 # pose change tolerance
    poselist = [_pose.position.x, _pose.position.y, _pose.position.z,
                _pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w]
    oldposelist = [_oldPose.position.x, _oldPose.position.y, _oldPose.position.z,
                   _oldPose.orientation.x, _oldPose.orientation.y, _oldPose.orientation.z, _oldPose.orientation.w]

    i = 0
    while i < 7:
        if(abs(oldposelist[i] - poselist[i]) > tolerance):
            print""
            print"poseChanged"
            print""
            return True
        i += 1
    return False

# gets pose of aruco marker and prints when it changed
def callback(data):
    global _oldPose
    global _pose
    _oldPose = _pose
    _pose = data.pose
    if(poseChanged()):
        marker_position = _pose.position
        marker_orientation = _pose.orientation
        print"Position:"
        print"x: %.4f" % marker_position.x, ", y: %.4f" % marker_position.y, ", z: %.4f" % marker_position.z

        print"Orientation:"
        print"x: %.4f" % marker_orientation.x, ", y: %.4f" % marker_orientation.y, ", z: %.4f" % marker_orientation.z, ", w: %.4f" % marker_orientation.w
        print""
        print""


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('aruco_subscriber', anonymous=True)

    rospy.Subscriber("/aruco_single/pose", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
