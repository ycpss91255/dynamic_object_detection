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
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "vehicle/odom_new", "odom/vlp_16"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot2odom");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    while (nh.ok()){
        tf_pose();
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    ros::spin();
}
