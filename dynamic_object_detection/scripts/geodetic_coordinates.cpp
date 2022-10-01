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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

float position[3];
std::queue <float> point_x;
std::queue <float> point_y;
std::queue <float> point_z;

// ROS_INFO("%d", out_x.size());
// delete [] array;

double Quaternion2Euler(float x, float y, float z, float w){
    double siny_cosp = 2.0*(w*z+x*y);
    double cosy_cosp = 1.0-2.0*(y*y+z*z);
    return atan2(siny_cosp, cosy_cosp);
}

void rotate(double x, double y, double z, double& out_x, double& out_y, double& out_z){
    double R_x = x ;
    double R_y = y ;
    // out_x = position[0] + R_x*cos(position[3]-M_PI/2) - R_y*sin(position[3]-M_PI/2);
    out_x = position[0] + R_x*cos(position[3]) - R_y*sin(position[3]);
    out_y = position[1] + R_x*sin(position[3]) + R_y*cos(position[3]);
    out_z = z;
}

void sub_pcl(const PointCloud::ConstPtr& msg){
    std::queue <float> temp_x;
    std::queue <float> temp_y;
    std::queue <float> temp_z;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        temp_x.push(pt.x), temp_y.push(pt.y), temp_z.push(pt.z);
    }
    point_x = temp_x;
    point_y = temp_y;
    point_z = temp_z;
}

void sub_odom(const nav_msgs::Odometry::ConstPtr& msg){
    position[0] = -msg->pose.pose.position.x;
    position[1] = -msg->pose.pose.position.y;
    position[2] = -msg->pose.pose.position.z;
    position[3] = Quaternion2Euler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void pub_pcl(std::queue <float> x,std::queue <float> y, std::queue <float> z,ros::Publisher pub_topic){
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "velodyne";
    msg->height = point_x.size();
    msg->width = 1;
    int i;
    while (!x.empty()){
        msg->points.push_back (pcl::PointXYZ(x.front(), y.front(), z.front()));
        x.pop();
        y.pop();
        z.pop();
    }
    if(point_x.size()==0){
        msg->points.push_back (pcl::PointXYZ(0.0, 0.0, 999.0));
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub_topic.publish (msg);
}

void point_transform(std::queue <float> x,std::queue <float> y, std::queue <float> z,std::queue <float>& out_x,std::queue <float>& out_y, std::queue <float>& out_z){
    double xx,yy,zz;
    out_x = {};
    out_y = {};
    out_z = {};
    while (!x.empty()){
        rotate(x.front(), y.front(), z.front(), xx, yy, zz);
        out_x.push(xx);
        out_y.push(yy);
        out_z.push(zz);
        x.pop();
        y.pop();
        z.pop();
    }
}

void tf_pose(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
    tf::Quaternion q;
    q.setRPY(0, 0, position[3]);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "odom/vlp_16"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "transform_pcl");
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub = nh.subscribe<PointCloud>("/velodyne_points", 1, sub_pcl);
    ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom_new", 1000, sub_odom);
    ros::Publisher pub = nh.advertise<PointCloud> ("/velodyne_points_tf", 1);
    ros::Rate loop_rate(10);
    while (nh.ok()){
        tf_pose();
        point_transform(point_x,point_y,point_z,point_x,point_y,point_z);
        pub_pcl(point_x,point_y,point_z,pub);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    ros::spin();
}
