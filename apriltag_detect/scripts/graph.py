#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from apriltag_detect.msg import graphing
from apriltag_detect.msg import error
import cv2
import numpy as np
import time


#fig, ax = plt.subplots()
#xdata, ydata = [], []
#ln, = plt.plot(xdata, ydata, 'r.')
plt.close('all')
data_array = []
error_current = 0
time_set = None

def init():
    ax.set_xlim(0, 1280)
    ax.set_ylim(0, 1024)
    return ln,

def update(data):
    xdata.append(data.point1_x)
    xdata.append(data.point2_x)
    xdata.append(data.point3_x)
    xdata.append(data.point4_x)

    ydata.append(data.point1_y)
    ydata.append(data.point2_y)
    ydata.append(data.point3_y)
    ydata.append(data.point4_y)

    ln.set_data(xdata, ydata)
    # ax.plot(xdata, ydata, 'b.')
    return ln,

def ErrorCallback(data):
    global error_current
    error_current = data.pose_estimation_error

def PlotCallback(data):
    global data_array
    global time_set
    global error_current
    time_set = time.time()
    data_array.append((data.point1_x, data.point1_y, error_current))
    data_array.append((data.point2_x, data.point2_y, error_current))
    data_array.append((data.point3_x, data.point3_y, error_current))
    data_array.append((data.point4_x, data.point4_y, error_current))
    #update(data)
    #plt.axis([0, 1280, 0, 1024])
    #plt.ylabel('pixels')
    #plt.xlabel('pixels')
    #plt.plot(data.point1_x, data.point1_y, '+', color='b')
    #plt.plot(data.point2_x, data.point2_y, '+', color='b')
    #plt.plot(data.point3_x, data.point3_y, '+', color='b')
    #plt.plot(data.point4_x, data.point4_y, '+', color='b')
    #plt.draw()
    #plt.pause(0.00001)

    """
    dist_coeffs = np.array([-0.405611, 0.137384, 0.000752, 0.000797, 0.0], dtype=np.float32)
    camera_matrix = np.array( [ [ 618.612671, 0.0, 663.505814],
                              [ 0.0, 692.588745, 471.725618 ],
                              [ 0.0, 0.0, 1.0] ], dtype=np.float32)
    image_points = np.array([(data.point1_x,data.point1_y), (data.point2_x,data.point2_y), (data.point3_x,data.point3_y), (data.point4_x,data.point4_y)], dtype=np.float32)
    board = [2, 2, 0.07]
    opts = []
    num_pts = board[0] * board[1]
    opts_loc = np.zeros((num_pts, 1, 3), dtype=np.float32)
    for j in range(num_pts):
        opts_loc[j, 0, 0] = (j/board[1])
        opts_loc[j ,0, 1] = (j % board[1])
        opts_loc[j, 0, 2] = 0
        opts_loc[j, 0, :] = opts_loc[j, 0, :] * board[2]
        opts.append(opts_loc)
    object_points = opts_loc
    object_points.astype('float32')
    ok, rot, trans = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
    #print ok
    #print rot
    #print trans
    rot.astype('float32')
    trans.astype('float32')
    rot3x3 , temp = cv2.Rodrigues(rot)
    object_points_world = np.asmatrix(rot3x3) * np.asmatrix(object_points.squeeze().T) + np.asmatrix(trans)
    reprojected_h = camera_matrix * object_points_world
    reproject = (reprojected_h[0:2, :] / reprojected_h[2, :])
    reprojection_errors = image_points.squeeze().T - reproject
    print'~~~~~~~~~~~~~~~~~~~~'
    #print image_points
    #print object_points_world
    #print reprojected_h
    #print reproject
    #print reprojection_errors
    reprojection_rms = np.sqrt(np.sum(np.array(reprojection_errors) ** 2) / np.product(reprojection_errors.shape))
    #print reprojection_rms
    print'~~~~~~~~~~~~~~~~~~~~'
    output = cv2.projectPoints(object_points, rot,trans, camera_matrix, dist_coeffs)
    print output
    """

def plotFunction(event):
    global data_array
    global time_set
    if(time_set == None):
        return
    if(time.time() - time_set > 10):
        array = np.array(data_array)
        plt.axis([0, 1280, 0, 1024])
        plt.ylabel('pixels')
        plt.xlabel('pixels')
        plt.scatter(array[:, 0], array[:, 1], marker="+", c=array[:, 2])
        cbar= plt.colorbar()
        cbar.set_label("error")
        plt.draw()
        plt.pause(0.001)
        plt.clf()

def main():
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber('/detector/graphing_points', graphing, PlotCallback, queue_size=100)
    rospy.Subscriber('/detector/error', error, ErrorCallback, queue_size=100)
    my_timer = rospy.Timer(rospy.Duration(1.0), plotFunction)
    # rospy.spin()
    plt.close('all')
    plt.ion()
    #plt.axis([0, 1280, 0, 1024])
    #plt.ylabel('pixels')
    #plt.xlabel('pixels')
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    main()
