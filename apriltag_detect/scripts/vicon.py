#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from apriltag_detect.msg import error
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import time
import tf

class Position:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = z
        self.z = y

class DronePosition:
    def __init__(self, landingpad_pose):
        self.vicon = Position()
        self.camera = Position()
        self.landingpad = Position(landingpad_pose.x, landingpad_pose.y, landingpad_pose.z)

    def viconCallBack(self, pose):
        self.vicon.x = pose.transform.translation.x - self.landingpad.x
        self.vicon.y = pose.transform.translation.y - self.landingpad.y
        self.vicon.z = pose.transform.translation.z - self.landingpad.z
        br = tf.TransformBroadcaster()
        br.sendTransform((self.vicon.x, self.vicon.y, self.vicon.z),
                 (tf.transformations.quaternion_from_euler(0, 0, 0)),
                 rospy.Time.now(),
                 "vicon",
                 "camera")

    def cameraCallBack(self, pose):
        self.camera.x = pose.pose.position.x
        self.camera.y = pose.pose.position.y
        self.camera.z = pose.pose.position.z

        error = abs(self.vicon.x - self.camera.x)
        error += abs(self.vicon.y - self.camera.y)
        error += abs(self.vicon.z - self.camera.z)

        rospy.loginfo("error:\t%f", error)
        rospy.loginfo("vicon:\t%f,\t%f,\t%f", self.vicon.x, self.vicon.y, self.vicon.z)
        rospy.loginfo("camera:\t%f,\t%f,\t%f", self.camera.x, self.camera.y, self.camera.z)


if __name__ == '__main__':
    drone = DronePosition(Position(-4.05, 0.43, 0.16))
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber('/detector/pose', PoseStamped, drone.cameraCallBack)
    rospy.Subscriber('/vicon/huan_minidrone2_UP_PLUS/huan_minidrone2_UP_PLUS', TransformStamped, drone.viconCallBack)
    rospy.spin()
