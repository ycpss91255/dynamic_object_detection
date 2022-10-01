#!/usr/bin/env python
# license removed for brevity

#sensor : Velodyne VLP-16
#   Sensing distance : 100m
#   frequency : 5~20Hz
#   Vertical angle range : +-15 degree
#   Vertical angle resolution : 2 degree
#   Horizontal angle resolution : 0.1~0.4 degree

from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sklearn.metrics import silhouette_score
from sklearn.cluster import KMeans
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# from numba import jit

import rospy
import numpy as np
import cv2
import time
import tf
import struct
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import math
import copy

ros_hz = 4

def calcu_dis(x,y,z,a,b,c):
    return ((x-a)*(x-a)+(y-b)*(y-b)+(z-c)*(z-c))**0.5

def box_rotate(x_center,y_center,angle,x,y,z):
    R_x = x - x_center
    R_y = y - y_center

    new_x = x_center + R_x*math.cos(angle*math.pi/180) - R_y*math.sin(angle*math.pi/180)
    new_y = y_center + R_x*math.sin(angle*math.pi/180) + R_y*math.cos(angle*math.pi/180)

    return (new_x,new_y,z)

def place2angle(start_x,start_y,end_x,end_y):
    xx = end_x-start_x
    yy = end_y-start_y
    out = None

    if xx==0 and yy==0:
        out = 999
    elif xx == 0:
        if yy>0:
            out = 90
        else:
            out = -90
    elif xx>0:
        out = math.degrees(math.atan(yy/xx))
    else:
        out = math.degrees(math.atan(yy/xx))+180

    return out

class object_detect():
    def __init__(self):
        self.sub_pcl = rospy.Subscriber("velodyne_points_tf", PointCloud2, self.set_point_cloud)
        self.sub_odom = rospy.Subscriber("odom/vlp_16", Odometry, self.set_odom)
        self.pub_pcl = rospy.Publisher("no_ground_points", PointCloud2, queue_size=10)
        self.pub_test_3d = rospy.Publisher("test_3d", PointCloud2, queue_size=10)
        self.pub_k_centers = rospy.Publisher("K_center", MarkerArray, queue_size=10)
        self.pub_test_2d = rospy.Publisher("test_2d", PointCloud2, queue_size=10)
        self.pub_box = rospy.Publisher("box_test", PointCloud2, queue_size=10)#for test
        self.pub_D_box = rospy.Publisher("dynamic_box", MarkerArray, queue_size=10)
        self.pub_S_box = rospy.Publisher("static_box", MarkerArray, queue_size=10)
        self.pub_img = rospy.Publisher('imagetimer', Image,queue_size=10)

        self.point_in = []
        self.point_filter = []
        self.point_without_ground = []
        self.pos = []
        self.front = 999

        self.detect_distance = 5.0 # meter
        self.detect_upper_limit = 0 # meter
        self.detect_lower_limit = -2 # meter
        self.detect_ground_range = 0.10 # meter
        self.ground_iteration = 4 #times
        self.lidar_height = 1.1 # meter
        self.box_points_max = 3 #meter

        self.K_means_center = []
        self.center_gap = 1.5
        self.place_record = [[999],[999],[999]]
        self.same_item_error = 0.6/ros_hz

        self.box_data = []
        self.dynamic_box = []
        self.static_box = []


    def set_point_cloud(self,msg):
        assert isinstance(msg, PointCloud2)
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        temp = []
        for p in gen:
            temp.append(p)
        self.point_in = temp

    def set_odom(self, msg):
        pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # quaternion to euler
        (Alpha, Beta, Gamma) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                            msg.pose.pose.orientation.y,
                                                                            msg.pose.pose.orientation.z,
                                                                            msg.pose.pose.orientation.w])
        front = Gamma * 180 / math.pi
        self.pos = [pos[0], pos[1], front]

    def Pub_point_cloud(self,pub_info):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        pub = pcl2.create_cloud_xyz32(header, pub_info)
        self.pub_pcl.publish(pub)

    def Pub_2D_test(self,pub_info):# for test
            if pub_info:
                data = [[]for i in range(len(pub_info))]
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = 'velodyne'
                fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('rgba', 12, PointField.UINT32, 1),
                ]

                for i in range(len(pub_info)):
                    data[i].append(pub_info[i][0])
                    data[i].append(pub_info[i][1])
                    data[i].append(0)
                    data[i].append(0xffff0000)

                pub = pcl2.create_cloud(header, fields, data)
                self.pub_test_2d.publish(pub)

    def Pub_3D_test(self,pub_info):# for test
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            pub = pcl2.create_cloud_xyz32(header, pub_info)
            self.pub_test_3d.publish(pub)

    def Pub_box(self,pub_info):# for test
        if pub_info:
            data = [[]for i in range(len(pub_info))]
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('rgba', 12, PointField.UINT32, 1),
            ]

            for i in range(len(pub_info)):
                data[i].append(pub_info[i][0])
                data[i].append(pub_info[i][1])
                data[i].append(0.3)
                data[i].append(0xffFF0000)

            pub = pcl2.create_cloud(header, fields, data)
            self.pub_box.publish(pub)

    def pub_centers(self,data):

        marker_array = MarkerArray()
        ground = 0

        # draw line
        for i in range(len(data)):

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()

            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.1

            marker.points = []
            marker.points.append(Point(data[i][0],data[i][1],ground))
            marker.points.append(Point(data[i][0],data[i][1],ground+2.5))

            marker_array.markers.append(marker)

        # remove topic data
        for i in range(len(data),10):

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()

            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.0
            marker.scale.x = 0.1

            marker.points = []
            marker.points.append(Point(0,0,ground))
            marker.points.append(Point(0,0,ground+2))

            marker_array.markers.append(marker)

        self.pub_k_centers.publish(marker_array)

    def pub_static_box(self,data):
        # 0 1 2 3     4     5     6
        # X Y Z Angle X_dis Y_dis height

        marker_array = MarkerArray()

        # draw line
        for i in range(len(data)):

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()

            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.1

            marker.points = []

            new_data = []
            j = 0

            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]))

            for j in range(16):
                marker.points.append(Point(new_data[j][0],new_data[j][1],new_data[j][2]))

            marker_array.markers.append(marker)

        # remove topic data
        for i in range(len(data),10):

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()

            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.0
            marker.scale.x = 0.0

            marker.points = []

            marker.points.append(Point(0,0,0))
            marker.points.append(Point(0,0,0+2))

            marker_array.markers.append(marker)

        self.pub_S_box.publish(marker_array)

    def pub_dynamic_box(self,data):
        # 0 1 2 3     4     5     6      7
        # X Y Z Angle X_dis Y_dis height direction

        marker_array = MarkerArray()

        for i in range(len(data)):
            # draw box
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()

            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.1

            marker.points = []

            new_data = []
            j = 0

            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]-data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]+data[i][6]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]-data[i][4],data[i][1]+data[i][5],data[i][2]))
            new_data.append(box_rotate(data[i][0],data[i][1],data[i][3],data[i][0]+data[i][4],data[i][1]+data[i][5],data[i][2]))

            for j in range(len(new_data)):
                marker.points.append(Point(new_data[j][0],new_data[j][1],new_data[j][2]))

            marker_array.markers.append(marker)

            # draw line
            direction_marker = Marker()
            direction_marker.header.frame_id = 'world'
            direction_marker.header.stamp = rospy.Time.now()

            direction_marker.id = len(data)+i
            direction_marker.action = Marker.ADD
            direction_marker.lifetime = rospy.Duration()
            direction_marker.type = Marker.LINE_STRIP
            direction_marker.pose.orientation.w = 1.0

            direction_marker.color.r = 1.0
            direction_marker.color.g = 0.0
            direction_marker.color.b = 0.0
            direction_marker.color.a = 1.0
            direction_marker.scale.x = 0.1

            direction_marker.points = []

            direction = []

            if data[i][7] != 999:
                direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+0,data[i][1]+  0,0))
                direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+1,data[i][1]+  0,0))
                direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+0.5,data[i][1]+0.25,0))
                direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+0.5,data[i][1]-0.25,0))
                direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+1,data[i][1]+  0,0))

                # direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+2,data[i][1]+  0,-1+0.5))
                # direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+2,data[i][1]+  0,-1-0.5))
                # direction.append(box_rotate(data[i][0],data[i][1],data[i][7],data[i][0]+3,data[i][1]+  0,-1))

            for j in range(len(direction)):
                direction_marker.points.append(Point(direction[j][0],direction[j][1],direction[j][2]))


            marker_array.markers.append(direction_marker)

        # remove topic data
        for i in range(len(data),10):
            # draw box
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()

            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration()
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0

            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.0
            marker.scale.x = 0.1

            marker.points = []

            marker.points.append(Point(0,0,0))
            marker.points.append(Point(0,0,2))

            # draw line
            direction_marker = Marker()
            direction_marker.header.frame_id = 'world'
            direction_marker.header.stamp = rospy.Time.now()

            direction_marker.id = len(data)+i
            direction_marker.action = Marker.ADD
            direction_marker.lifetime = rospy.Duration()
            direction_marker.type = Marker.LINE_STRIP
            direction_marker.pose.orientation.w = 1.0

            direction_marker.color.r = 0.0
            direction_marker.color.g = 0.0
            direction_marker.color.b = 0.0
            direction_marker.color.a = 0.0
            direction_marker.scale.x = 0.1

            direction_marker.points = []

            direction_marker.points.append(Point(0,0,0))
            direction_marker.points.append(Point(0,0,2))


            marker_array.markers.append(direction_marker)


        self.pub_D_box.publish(marker_array)

    def points_filter(self,data):# no need

        i = 0
        temp = []

        rf = []
        lf = []
        bk = []
        rfed = []
        lfed = []
        bked = []

        rfsum = 0
        lfsum = 0
        bksum = 0

        rf_x = 0
        rf_y = 0
        rf_z = 0

        lf_x = 0
        lf_y = 0
        lf_z = 0

        bk_x = 0
        bk_y = 0
        bk_z = 0

        pa = []
        pb = []
        pc = []

        a = 0
        b = 0
        c = 0
        k = 0

        #take away points that out of range
        for i in data:
            if (calcu_dis(i[0],i[1],0,self.pos[0],self.pos[1],0) <= self.detect_distance):
                if (i[0]>self.pos[0] and i[1]<self.pos[1]):
                    rf.append([i[0],i[1],i[2]+self.lidar_height])
                elif(i[0]>self.pos[0] and i[1]>self.pos[1]):
                    lf.append([i[0],i[1],i[2]+self.lidar_height])
                elif(i[0]<self.pos[0]):
                    bk.append([i[0],i[1],i[2]+self.lidar_height])

        #remove ground points
        for j in range(self.ground_iteration):
            for i in range(len(rf)):
                rfsum = rfsum + rf[i][2]
            if (len(rf) != 0):
                rfsum = rfsum/len(rf)
            for i in range(len(rf)):
                if (rf[i][2]<rfsum):
                    rfed.append(rf[i])
            rf = rfed

            for i in range(len(lf)):
                lfsum = lfsum + lf[i][2]
            if (len(lf) != 0):
                lfsum = lfsum/len(lf)
            for i in range(len(lf)):
                if (lf[i][2]<lfsum):
                    lfed.append(lf[i])
            lf = lfed

            for i in range(len(bk)):
                bksum = bksum + bk[i][2]
            if (len(bk) != 0):
                bksum = bksum/len(bk)
            for i in range(len(bk)):
                if (bk[i][2]<bksum):
                    bked.append(bk[i])
            bk = bked

        for i in range(len(rf)):
            rf_x = rf_x + rf[i][0]
            rf_y = rf_y + rf[i][1]
            rf_z = rf_z + rf[i][2]

        for i in range(len(lf)):
            lf_x = lf_x + lf[i][0]
            lf_y = lf_y + lf[i][1]
            lf_z = lf_z + lf[i][2]

        for i in range(len(bk)):
            bk_x = bk_x + bk[i][0]
            bk_y = bk_y + bk[i][1]
            bk_z = bk_z + bk[i][2]

        if len(rf) == 0:
            rf = [0]
        if len(lf) == 0:
            lf = [0]
        if len(bk) == 0:
            bk = [0]
        pa = [lf_x/len(lf),lf_y/len(lf),lf_z/len(lf)]
        pb = [bk_x/len(bk),bk_y/len(bk),bk_z/len(bk)]
        pc = [rf_x/len(rf),rf_y/len(rf),rf_z/len(rf)]

        a = (pc[1]-pb[1])*(pa[2]-pb[2])-(pc[2]-pb[2])*(pa[1]-pb[1])
        b = (pc[2]-pb[2])*(pa[0]-pb[0])-(pa[2]-pb[2])*(pc[0]-pb[0])
        c = (pc[0]-pb[0])*(pa[1]-pb[1])-(pa[0]-pb[0])*(pc[1]-pb[1])
        k = ((pc[1]-pb[1])*(pa[2]-pb[2])-(pc[2]-pb[2])*(pa[1]-pb[1]))*pa[0]+((pc[2]-pb[2])*(pa[0]-pb[0])-(pa[2]-pb[2])*(pc[0]-pb[0]))*pa[1]+((pc[0]-pb[0])*(pa[1]-pb[1])-(pa[0]-pb[0])*(pc[1]-pb[1]))*pa[2]

        if pa[2] == 0:
            if pb[2]>pc[2]:
                pa[2] = pb[2]
            else:
                pa[2] = pc[2]
        if pb[2] == 0:
            if pa[2] > pc[2]:
                pb[2] = pa[2]
            else:
                pb[2] = pc[2]
        if pc[2] == 0:
            if pa[2]>pb[2]:
                pc[2] = pa[2]
            else:
                pc[2] = pb[2]

        print (pa[2],pb[2],pc[2])

        if a==0 or b==0 or c==0:
            for i in data:
                if(self.detect_ground_range*2<=i[2]+self.lidar_height and calcu_dis(i[0],i[1],0,self.pos[0],self.pos[1],0) <= self.detect_distance):
                    temp.append([i[0],i[1],i[2]+self.lidar_height])
        else:
            for i in data:
                if((k-a*i[0]-b*i[1])/c+self.detect_ground_range<=i[2]+self.lidar_height and calcu_dis(i[0],i[1],0,self.pos[0],self.pos[1],0) <= self.detect_distance):
                    temp.append([i[0],i[1],i[2]+self.lidar_height])
        return temp

    def points_filter2(self,data):
        temp = []
        #take away points that out of range
        for i in data:
            if (calcu_dis(i[0],i[1],0,self.pos[0],self.pos[1],0) <= self.detect_distance and self.detect_ground_range*2<i[2]+self.lidar_height):
                temp.append([i[0],i[1],i[2]+self.lidar_height])
        return temp

    def pub_image(self,data):
        self.pub_img.publish(CvBridge().cv2_to_imgmsg(data,"bgr8"))

    def reduce_2d_points(self,data):# no need
        i = 0
        reduce_n = 30
        detect_distance = self.detect_distance

        temp = [[0]*reduce_n for i in range(reduce_n)]
        out = []

        for i in data:
            for j in range(reduce_n):
                for k in range(reduce_n):
                    if detect_distance-detect_distance*2/reduce_n*k>=i[1] and i[1]>=detect_distance-detect_distance*2/reduce_n*(k+1):
                        if detect_distance-detect_distance*2/reduce_n*j>=i[0] and i[0]>=detect_distance-detect_distance*2/reduce_n*(j+1):
                            if temp[j][k] <=25:
                                temp[j][k] = temp[j][k]+1

        for _ in range(reduce_n):
            print (temp[_])

        for j in range(reduce_n):
            for k in range(reduce_n):
                if temp[j][k] >= 1:
                    out.append([detect_distance-detect_distance*2/reduce_n*(j+0.5), detect_distance-detect_distance*2/reduce_n*(k+0.5)])

                    # F B L R
                    out.append([detect_distance-detect_distance*2/reduce_n*(j+0.8), detect_distance-detect_distance*2/reduce_n*(k+0.5)])
                    out.append([detect_distance-detect_distance*2/reduce_n*(j+0.2), detect_distance-detect_distance*2/reduce_n*(k+0.5)])
                    out.append([detect_distance-detect_distance*2/reduce_n*(j+0.5), detect_distance-detect_distance*2/reduce_n*(k+0.8)])
                    out.append([detect_distance-detect_distance*2/reduce_n*(j+0.5), detect_distance-detect_distance*2/reduce_n*(k+0.2)])

                    # # LF LB RF RB
                    # out.append([detect_distance-detect_distance*2/reduce_n*(j+0.0), detect_distance-detect_distance*2/reduce_n*(k+0.0)])
                    # out.append([detect_distance-detect_distance*2/reduce_n*(j+0.0), detect_distance-detect_distance*2/reduce_n*(k+0.1)])
                    # out.append([detect_distance-detect_distance*2/reduce_n*(j+1.0), detect_distance-detect_distance*2/reduce_n*(k+0.0)])
                    # out.append([detect_distance-detect_distance*2/reduce_n*(j+1.0), detect_distance-detect_distance*2/reduce_n*(k+1.0)])


        # print(len(data),len(out))
        # for i in range(reduce_n):
        #     print(temp[i])
        # print("----")

        return out

    def point2img(self,data):
        # tt = time.time()

        detect_distance = self.detect_distance
        img_size = 40 # 60*60
        break_flag = False
        temp = [[0]*img_size for _ in range(img_size)]

        for i in data:
            for j in range(img_size):
                for k in range(img_size):
                    if self.pos[0]-detect_distance+detect_distance*2/img_size*k<i[0] and i[0]<=self.pos[0]-detect_distance+detect_distance*2/img_size*(k+1):
                        if self.pos[1]+detect_distance-detect_distance*2/img_size*j>i[1] and i[1]>=self.pos[1]+detect_distance-detect_distance*2/img_size*(j+1):
                            temp[j][k] = 255
                            break_flag = True
                            break
                if break_flag:
                    break_flag = False
                    break

        data = np.resize(temp,(img_size,img_size,1))
        cv2.imwrite("src/dynamic_object_detection/image/aaa.jpg",data)
        img_pub = cv2.imread("src/dynamic_object_detection/image/aaa.jpg")
        self.pub_image(img_pub)

        # print('Time used: {} sec'.format(time.time()-tt))

    def K_means_clustering(self,data):
        # tt = time.time()
        dx = []
        temp = []

        if data != []:
            for i in data:
                temp = (i[0], i[1])
                dx.append(temp)

            # K value range (2~10)
            k_range = range(2, 10)

            dx = np.array(dx)

            scores = []

            # Record the effectiveness of the KMeans model built by each K value
            for i in k_range:
                kmeans = KMeans(n_clusters=i).fit(dx)
                scores.append(silhouette_score(dx, kmeans.predict(dx)))# Silhouette factor

            # Find the largest silhouette coefficient to determine the K value
            selected_K = scores.index(max(scores)) + 2

            # Rebuild the KMeans model and predict the target value
            kmeans = KMeans(n_clusters=selected_K).fit(dx)

            # Data center point of the new group
            centers = kmeans.cluster_centers_
            # print('Time used: {} sec'.format(time.time()-tt))

            return centers
        else:
          return []

    def center_filter(self,data):
        temp = 0
        # for i in range(len(data)-1):
        #     if calcu_dis(data[i][0],data[i][1],0,data[i+1][0],data[i+1][1],0)<self.center_gap:

        return data

    def place_matching(self,data):
        if not np.array_equal(data,[]):
            data = list(data)
            self.place_record[0] = self.place_record[1]

            if self.place_record[0] == [999]:
                self.place_record[1] = data
            else:
                bb = self.place_record[0]
                aa = data

                aa_dict = {}
                bb_dict = {}

                if len(aa) <=len(bb):
                    for i in range(len(bb) - len(aa)):
                        aa.append(np.array([99.9, 99.9]))
                elif len(bb) <=len(aa):
                    for i in range(len(aa) - len(bb)):
                        bb.append(np.array([99.9, 99.9]))

                # placedis
                for i in range(len(aa)):
                    temp_aa = []
                    temp_bb = []
                    for j in range(len(bb)):
                        temp_aa.append(calcu_dis(aa[i][0],aa[i][1],0,bb[j][0],bb[j][1],0))
                        temp_bb.append(calcu_dis(bb[i][0],bb[i][1],0,aa[j][0],aa[j][1],0))
                    aa_dict[i] = temp_aa
                    bb_dict[i] = temp_bb

                # dis2rank
                for i in range(len(aa)):
                    temp_aa = [0 for _ in range(len(bb))]
                    temp_bb = [0 for _ in range(len(bb))]
                    for j in range(len(bb)):
                        min_aa = 999
                        min_bb = 999
                        aa_flag = 0
                        bb_flag = 0
                        for _ in range(len(bb)):
                            if aa_dict[i][_]<min_aa:
                                min_aa = aa_dict[i][_]
                                aa_flag = _
                        for _ in range(len(aa)):
                            if bb_dict[i][_]<min_bb:
                                min_bb = bb_dict[i][_]
                                bb_flag = _
                        aa_dict[i][aa_flag] = 999
                        bb_dict[i][bb_flag] = 999
                        temp_aa[j] = aa_flag
                        temp_bb[j] = bb_flag
                    aa_dict[i] = temp_aa
                    bb_dict[i] = temp_bb

                guyprefers = aa_dict
                galprefers = bb_dict

                guys = sorted(guyprefers.keys())
                gals = sorted(galprefers.keys())

                def matchmaker():
                    guysfree = guys[:]
                    engaged  = {}
                    guyprefers2 = copy.deepcopy(guyprefers)
                    galprefers2 = copy.deepcopy(galprefers)
                    while guysfree:
                        guy = guysfree.pop(0)
                        guyslist = guyprefers2[guy]
                        gal = guyslist.pop(0)
                        fiance = engaged.get(gal)
                        if fiance == None:
                            # She's free
                            engaged[gal] = guy
                            # print("  %s and %s" % (guy, gal))
                        else:
                            # The bounder proposes to an engaged lass!
                            galslist = galprefers2[gal]
                            if galslist.index(fiance) > galslist.index(guy):
                                # She prefers new guy
                                engaged[gal] = guy
                                # print("  %s dumped %s for %s" % (gal, fiance, guy))
                                if guyprefers2[fiance]:
                                    # Ex has more girls to try
                                    guysfree.append(fiance)
                            else:
                                # She is faithful to old fiance
                                if guyslist:
                                    # Look again
                                    guysfree.append(guy)
                    return engaged

                engaged = matchmaker()

                # swap
                temp = []
                for i in range(len(aa)):
                    temp.append(data[engaged[i]])
                self.place_record[1] = temp

                # print (self.place_record[0])
                # print (self.place_record[1])

        else:
            self.place_record[0] = [999]
            self.place_record[1] = [999]
            self.place_record[2] = [999]

    def direction_prediction(self,data):
        # for i in range(len(data[0])):
        #     print(data[0][i],data[1][i])

        print('---')

    def Rotating_caliper(self,data,center):
        box_ground = 0
        box_data = []

        def rotation(data,center):
            angle_difference = 5
            find_min_degree = [] # max_x min_x max_x-min_x degree_x
            min_degree = [0,0,999,0]
            rotated_data_y = []
            max_z = -999

            for D in range(0,180,angle_difference):
                max_x = -999.0
                min_x = 999.0
                max_y = -999.0
                min_y = 999.0
                rotated_data = []

                #rotate points
                for i in range(len(data)):
                    rotated_data.append(box_rotate(center[0],center[1],D,data[i][0],data[i][1],0))

                #find x degree
                for i in range(len(rotated_data)):
                    if max_x <=rotated_data[i][0]:
                        max_x = rotated_data[i][0]
                    if rotated_data[i][0] <= min_x:
                        min_x = rotated_data[i][0]

                find_min_degree.append((max_x,min_x,max_x-min_x,D))

            for i in range(int(180/angle_difference)):
                if min_degree[2] >= find_min_degree[i][2]:
                    min_degree = find_min_degree[i]

            #find y max min & z max
            for i in range(len(data)):
                rotated_data_y.append(box_rotate(center[0],center[1],min_degree[3],data[i][0],data[i][1],0))
                if max_z <= data[i][2]:
                    max_z = data[i][2]

            for i in range(len(rotated_data_y)):
                if max_y <=rotated_data_y[i][1]:
                    max_y = rotated_data_y[i][1]
                if rotated_data_y[i][1] <= min_y:
                    min_y = rotated_data_y[i][1]

            return(((min_degree[0]+min_degree[1])/2,(max_y+min_y)/2,box_ground,
                                    -min_degree[3],(min_degree[0]-min_degree[1])/2,(max_y-min_y)/2,max_z))

        # find points around center
        if not np.array_equal(data,[]) and not np.array_equal(center,[]):
            for i in range(len(center)):
                k = 0.5
                inbox_s = []
                inbox_l = []
                for w in range(15):
                    for j in data:
                        if calcu_dis(j[0],j[1],0,center[i][0],center[i][1],0)<k:
                            inbox_s.append(j)
                    k=k+0.1
                    for j in data:
                        if calcu_dis(j[0],j[1],0,center[i][0],center[i][1],0)<k:
                            inbox_l.append(j)

                    if (len(inbox_l) <= len(inbox_s)):
                        break

                    inbox_s = []
                    inbox_l = []
                box_data.append(rotation(inbox_l,center[i]))

        return box_data

    def Dynamic_or_static(self,data, place):
        # 0 1 2 3     4     5     6      7
        # X Y Z Angle X_dis Y_dis height direction
        Dynamic = []
        static = []
        direction = []

        if place != [999]:

            for i in range(len(data)):
                place[i] = list(place[i])
                # print (place[i][0], place[i][1], data[i][0],data[i][1], calcu_dis(place[i][0],place[i][1],0,data[i][0],data[i][1],0))

                if place[i][0]!=99 and place[i][1]!=99 and data[i][0]!=0 and data[i][1]!=0:
                    # print (place[i])
                    # print (data[i][0],data[i][1])
                    if calcu_dis(place[i][0],place[i][1],0,data[i][0],data[i][1],0)>self.same_item_error:
                        direction = place2angle(place[i][0],place[i][1],data[i][0],data[i][1])
                        Dynamic.append([data[i][0], data[i][1],data[i][2], data[i][3], data[i][4], data[i][5], data[i][6], direction])
                    else:
                        static.append(data[i])

        return static, Dynamic

    def process(self):

        # remove ground points
        self.point_without_ground = self.points_filter2(self.point_in)
        self.Pub_point_cloud(self.point_without_ground)

        # points to image for yolo data
        # self.point2img(self.point_without_ground)

        # K-means clustering
        self.K_means_center = self.center_filter(self.K_means_clustering(self.point_without_ground))
        self.pub_centers(self.K_means_center)

        # plcae record & matching & predict
        self.place_matching(self.K_means_center)

        # # next place prediction
        self.direction_prediction(self.place_record)

        # Rotating Calipers
        self.box_data = self.Rotating_caliper(self.point_without_ground,self.place_record[1])

        # Dynamic or static
        self.static_box, self.dynamic_box = self.Dynamic_or_static(self.box_data, self.place_record[0])

        # pub bounding box
        self.pub_static_box(self.static_box)
        self.pub_dynamic_box(self.dynamic_box)

        # print data length
        print(len(self.point_in),len(self.point_without_ground),len(self.K_means_center))

def main():
    rospy.init_node('object_detect', anonymous=True)
    rate = rospy.Rate(ros_hz) # 10hz
    while not rospy.is_shutdown():
        object_detect.process()
        rate.sleep()

if __name__ == '__main__':
    try:
        object_detect = object_detect()
        main()
    except rospy.ROSInterruptException:
        pass
