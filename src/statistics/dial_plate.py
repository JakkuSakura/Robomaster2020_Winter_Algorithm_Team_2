#!/bin/python2
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import thread
import graph
import math
import threading
from collections import deque

shown_time = 30
time_window = 60
odom_rate = 10
mutex = threading.Lock()

poses = deque(maxlen=time_window * odom_rate)
ticks = deque(maxlen=time_window * odom_rate)
x_pose = deque(maxlen=time_window * odom_rate)
y_pose = deque(maxlen=time_window * odom_rate)
speed = deque(maxlen=time_window * odom_rate)
acc = deque(maxlen=time_window * odom_rate)


def odometryCb(msg):
    if len(ticks) > 1 and msg.header.stamp.to_sec() - ticks[-1] < 0.1 and msg.pose.pose.position.x == x_pose[-1] and msg.pose.pose.position.y == y_pose[-1]:
        return

    with mutex:
        poses.append(msg)
        ticks.append(msg.header.stamp.to_sec())
        x_pose.append(msg.pose.pose.position.x)
        y_pose.append(msg.pose.pose.position.y)
        if len(ticks) >= 2:
            delta = ticks[-1] - ticks[-2]
            speed.append(math.hypot(
                (x_pose[-1] - x_pose[-2])/delta, (y_pose[-1] - y_pose[-2])/delta))
            acc.append((speed[-1] - speed[-2])/delta)
        else:
            speed.append(float('nan'))
            acc.append(float('nan'))


def listener():
    rospy.init_node('listener', anonymous=True, disable_signals=True)
    rospy.Subscriber("/odom", Odometry, odometryCb)
    rospy.spin()

class MyDataset(graph.Dataset):
    def __init__(self, title, x, y):
        super(MyDataset, self).__init__(title, x, y)
    
    def range(self):
        if len(self.x) >= shown_time * odom_rate:
            return (self.x[-1] - shown_time, self.x[-1])
        else:
            return (self.x[0], self.x[-1])

if __name__ == '__main__':
    thread.start_new_thread(listener, ())
    datasets = [
        MyDataset("Speed", ticks, speed),
        MyDataset("Acceleration", ticks, acc),
    ]
    graph.main("ROS Monitor", datasets, mutex)
