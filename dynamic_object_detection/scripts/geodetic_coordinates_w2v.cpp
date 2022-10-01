#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"

void tf_pose(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.4) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom/vlp_16", "velodyne"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "transform_pcl_w2v");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    while (nh.ok()){
        tf_pose();
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    ros::spin();
}
