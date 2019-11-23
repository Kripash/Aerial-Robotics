#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from apriltag_detect.msg import graphing

#fig, ax = plt.subplots()
#xdata, ydata = [], []
#ln, = plt.plot(xdata, ydata, 'r.')

def init():
    ax.set_xlim(0, 1024)
    ax.set_ylim(0, 1280)
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

def PlotCallback(data):
    #update(data)
    plt.plot(data.point1_x, data.point1_y, '*')
    plt.plot(data.point2_x, data.point2_y, '*')
    plt.plot(data.point3_x, data.point3_y, '*')
    plt.plot(data.point4_x, data.point4_y, '*')
    #plt.axis("axis")
    plt.draw()
    plt.pause(0.00001)
    # ani = FuncAnimation(fig, update, fargs=data, init_func=init, interval=1000)
    #plt.show()

def main():
    rospy.init_node('graph', anonymous=True)
    rospy.Subscriber('/detector/graphing_points', graphing, PlotCallback, queue_size=5)
    # rospy.spin()
    plt.ion()
    #plt.xlim((0, 1024))
    #plt.ylim((0, 1280))
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    main()
