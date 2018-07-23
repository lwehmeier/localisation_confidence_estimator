#!/usr/bin/python
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16
import struct
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import numpy as np
class VL6180:
    def __init__(self, angle):
        self.distance = 255
        self.res = 0.1
        self.angle = angle
    def update(self, grid):
        angle = self.angle
        cx = len(grid)/2
        cy = len(grid[0])/2
        if angle == "lf":
            tiles = [(cx + 1, cy), (cx + 1, cy+1), (cx +2, cy+2)]
        if angle == "lr":
            tiles = [(cx + 1, cy), (cx + 1, cy-1), (cx +2, cy-2)]
        if angle == "rf":
            tiles = [(cx - 1, cy), (cx - 1, cy+1), (cx -2, cy+2)]
        if angle == "rr":
            tiles = [(cx - 1, cy), (cx - 1, cy-1), (cx -2, cy-2)]
        if angle == "fl":
            tiles = [(cx, cy+1), (cx+1, cy+1), (cx +2, cy+2)]
        if angle == "fr":
            tiles = [(cx, cy+1), (cx-1, cy+1), (cx -2, cy+2)]
        if angle == "rl":
            tiles = [(cx, cy-1), (cx+1, cy-1), (cx +2, cy-2)]
        if angle == "rr":
            tiles = [(cx, cy-1), (cx-1, cy-1), (cx -2, cy-2)]
        for i in range(0,3):
            if grid[tiles[i][0], tiles[i][1]] > 90:
                self.distance = 50 + 100*i
                self.interrupt()
                return
        self.distance = 255
    def interrupt(self):
        print("detected obstacle. Stopping platform..")
        rospy.Publisher("/platform/e_stop", Int16, queue_size=1).publish(Int16(1))
    def setGridRes(self, res):
        self.res = res


vl_array = [
    VL6180("lf"),
    VL6180("lr"),
    VL6180("rl"),
    VL6180("rr"),
    VL6180("fl"),
    VL6180("fr"),
    VL6180("rl"),
    VL6180("rr")
    ]

def callbackTimer(event):
    for vl in vl_array:
        rospy.Publisher("/platform/distance/"+str(vl.angle), Int16, queue_size = 1).publish(Int16(vl.distance))
def callbackCM(costmap):
    global cmap
    cmap = costmap
    m = grid2matrix(costmap)
    for vl in vl_array:
        vl.update(m)
def callbackUpdate(costmap_u):
    if costmap_u.x != 0 or costmap_u.y != 0:
        print("cannot handle costmap update")
        return
    global cmap
    cmap.data = costmap_u.data
    callbackCM(cmap)

def grid2matrix(occupancy_grid):
    res = occupancy_grid.info.resolution
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = occupancy_grid.data
    matrix = []
    for x in range(0,width):
        matrix.append([])
        for y in range(0,height):
            mval = data[x*width + y]
            matrix[x].append(mval)
    return np.array(matrix)

rospy.init_node('localisation_confidence_estimator')
rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callbackCM, queue_size=5)
rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGridUpdate, callbackUpdate, queue_size=5)
rospy.Timer(rospy.Duration(0.5), callbackTimer)
r = rospy.Rate(10) # 10hz
rospy.spin()
