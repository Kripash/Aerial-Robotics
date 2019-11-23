#!/usr/bin/env python

import rospy
from apriltag_detect.msg import graphing

def ImagePlotCallback(data):
    pass

if __name__ == '__main__':
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber('/image_point', graphing, ImagePlotCallback, 1)
    rospy.spin()
