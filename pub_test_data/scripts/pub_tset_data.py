#!/usr/bin/env python
# license removed for brevity
import rospy
import sensor_msgs.point_cloud2 as pcl2
import os
import numpy as np
import pandas as pd
import tf

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix

FRAME_ID = 'velodyne'

DATA_PATH = "./src/pub_test_data/test_deta/2011_09_26/2011_09_26_drive_0005_sync/"

IMU_COLUMNS_NAMES = ['lat', 'lon', 'alt', 'roll', 'pitch', 'yaw', 'vn', 've', 'vf','vl', 'vu', 'ax', 'ay', 'az', 'af', 'al', 'au',
                     'wx', 'wy', 'wz', 'wf', 'wl', 'wu','posacc', 'valacc', 'navstat' ,'numsats', 'posmod', 'velmode', 'orimode']

def pub_test_data():
    frame = 0
    rospy.init_node('simulation_points', anonymous=True)
    pcl_pub = rospy.Publisher('velodyne_points', PointCloud2, queue_size=10)
    imu_pub = rospy.Publisher('simulation_imu', Imu, queue_size=10)
    gps_pub = rospy.Publisher('simulation_gps', NavSatFix, queue_size=10)
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        # publish point_cloud data
        point_cloud = np.fromfile(os.path.join(DATA_PATH, 'velodyne_points/data/%010d.bin'%13), dtype=np.float32).reshape(-1, 4)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = FRAME_ID

        pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))


        # publish imu data
        imu_data = pd.read_csv(os.path.join(DATA_PATH, 'oxts/data/%010d.txt'%frame), header=None, sep=' ')
        imu_data.columns = IMU_COLUMNS_NAMES

        imu = Imu()
        imu.header.frame_id = FRAME_ID
        imu.header.stamp = rospy.Time.now()

        q = tf.transformations.quaternion_from_euler(float(imu_data.roll),float(imu_data.pitch),float(imu_data.yaw))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = imu_data.af
        imu.linear_acceleration.y = imu_data.al
        imu.linear_acceleration.z = imu_data.au
        imu.angular_velocity.x = imu_data.wf
        imu.angular_velocity.y = imu_data.wl
        imu.angular_velocity.z = imu_data.wu

        imu_pub.publish(imu)


        # publish gps data
        gps = NavSatFix()
        gps.header.frame_id = FRAME_ID
        gps.header.stamp = rospy.Time.now()

        gps.latitude = imu_data.lat
        gps.longitude = imu_data.lon
        gps.altitude = imu_data.alt

        gps_pub.publish(gps)

        rospy.loginfo("published")
        rate.sleep()
        frame += 1
        frame %=154



if __name__ == '__main__':
    try:
        pub_test_data()

    except rospy.ROSInterruptException:
        pass
