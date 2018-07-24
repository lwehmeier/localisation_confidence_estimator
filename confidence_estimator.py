#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16
from geometry_msgs.msg import Pose, PoseArray
import struct
import numpy as np
class Estimator:
    def __init__(self):
        pass
    def update(self, array):
        pass

        
def callbackTimer(event):
    pass
def callbackPC(poseArray):
    mean_x = 0
    var_x = 0
    mean_y = 0
    var_y = 0
    for pose in poseArray.poses:
        mean_x += pose.position.x
        mean_y += pose.position.y
    mean_x /= len(poseArray.poses)
    mean_y /= len(poseArray.poses)
    for pose in poseArray.poses:
        var_x += (pose.position.x - mean_x) * (pose.position.x - mean_x)
        var_y += (pose.position.y - mean_y) * (pose.position.y - mean_y)
    var_x /= len(poseArray.poses)-1
    var_y /= len(poseArray.poses)-1
    print("n: " + str(len(poseArray.poses)) + " mean: (" + str(mean_x) +", " + str(mean_y)+ ") variance: (" + str(var_x) + ", " + str(var_y) + ")")

rospy.init_node('localisation_confidence_estimator')
rospy.Subscriber("/particlecloud", PoseArray, callbackPC, queue_size=5)
rospy.Timer(rospy.Duration(0.5), callbackTimer)
r = rospy.Rate(10) # 10hz
rospy.spin()
