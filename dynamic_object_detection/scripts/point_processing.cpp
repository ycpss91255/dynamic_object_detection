#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Odometry.h"
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <ctime>
#include <typeinfo>

using namespace std;

int ros_hz = 4;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

queue <float> point_x, point_y, point_z;
double position[3];

// parameter for real world
double lidar_height = 0.35; // meters
int Ransac_iter_times = 1000; // times
double Ransac_range = 0.15; // meters
double Ground_error = 0.15; // meters
double Ground_plane_error = 20.0; // degree
double Wall_plane_error = 70.0; // degree
double Wall_error = 0.15; // meters
double Wall_min_size = 2*1; // meters*meters
double wall_min_density = 0.6; // percent/100
double detect_range = 1.5; // meters
int point_sim_pieces = 40;  // pieces*pieces
int lattice_min_density = 2; // points per square
int rotate_angle_gap = 10;  // degrees
double box_error_endure = 0.05; // meters
int max_object_number = 10;
double static_to_unstable_error = 0.3; // meters/sec
double unstable_to_dynanic_error = 0.6; // meters/sec
double same_item_error = 10.0; // meters/sec
double big_item_error = 0.3; // meters
int max_wall_num = 4;
int wall_slice_pieces = 20;  // pieces*pieces
double k_center_line_wifth = 1.5; // meter
double dynamic_max_speed = 5.0; // meters/sec


// // parameter for gazebo simulator
// double lidar_height = 1.1; // meters
// int Ransac_iter_times = 1000; // times
// double Ransac_range = 0.15; // meters
// double Ground_error = 0.2; // meters
// double Ground_plane_error = 10.0; // degree
// double Wall_plane_error = 80.0; // degree
// double Wall_error = 0.2; // meters
// double Wall_min_size = 2*3; // meters*meters
// double wall_min_density = 0.6; // percent/100
// double detect_range = 5.5; // meters
// int point_sim_pieces = 40;  // pieces*pieces
// int lattice_min_density = 2; // points per square
// int rotate_angle_gap = 10;  // degrees
// double box_error_endure = 0.2; // meters
// int max_object_number = 10;
// double static_to_unstable_error = 0.8; // meters/sec
// double unstable_to_dynanic_error = 1.2; // meters/sec
// double same_item_error = 10.0; // meters/sec
// double big_item_error = 1.4; // meters
// int max_wall_num = 4;
// int wall_slice_pieces = 20;  // pieces*pieces
// double k_center_line_wifth = 2.5; // meter
// double dynamic_max_speed = 5.0; // meters/sec

queue <float> place_record_1_x = {}; // last time
queue <float> place_record_1_y = {}; // last time
queue <float> place_record_2_x = {}; // last last time
queue <float> place_record_2_y = {}; // last last time
queue <float> place_record_3_x = {}; // last last last time
queue <float> place_record_3_y = {}; // last last last time

queue <float> test_x_p, test_y_p, test_z_p;

double Cal_dis(double x,double y,double z, double a, double b, double c){
    return sqrt((x-a)*(x-a)+(y-b)*(y-b)+(z-c)*(z-c));
}

double cal_sse(double *x, double *y, int dim){
    int i;
    double t, sum=0.0;
    for(i=0; i<dim; ++i){
        t=x[i]-y[i], sum+=t*t;
    }
    return sum;
}

double Quaternion2Euler(float x, float y, float z, float w){
    double siny_cosp = 2.0*(w*z+x*y);
    double cosy_cosp = 1.0-2.0*(y*y+z*z);
    return atan2(siny_cosp, cosy_cosp);
}

double Box_rotate_x(double x_center,double y_center,double angle,double x,double y){
    double R_x = x - x_center;
    double R_y = y - y_center;

    double new_x = x_center + R_x*cos(angle*M_PI/180) - R_y*sin(angle*M_PI/180);
    // double new_y = y_center + R_x*sin(angle*M_PI/180) + R_y*cos(angle*M_PI/180);

    return new_x;
}

double Box_rotate_y(double x_center,double y_center,double angle,double x,double y){
    double R_x = x - x_center;
    double R_y = y - y_center;

    // double new_x = x_center + R_x*cos(angle*M_PI/180) - R_y*sin(angle*M_PI/180);
    double new_y = y_center + R_x*sin(angle*M_PI/180) + R_y*cos(angle*M_PI/180);

    return new_y;
}

double angle_with_plane_and_plane(double x1, double y1, double z1,double x2, double y2, double z2){
    return acos((x1*x2+y1*y2+z1*z2)/(sqrt(x1*x1+y1*y1+z1*z1)*sqrt(x2*x2+y2*y2+z2*z2)))*180/M_PI;
}

double point_to_plane_distance(double a, double b, double c, double d, double x, double y, double z){
    // positive => above plane
    // negative => below plane
    return abs((a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c));
}

double project_point_to_plane(double a, double b, double c, double d, double x, double y, double z, int options){
    double t = (-d-a*x-b*y-c*z)/(a*a+b*b+c*c);
    if(options==0){
        return x + a*t;
    }else if(options==1){
        return y + b*t;
    }else if(options==2){
        return z + c*t;
    }else{
        cout<<"project_point_to_plane error";
        return 0;
    }
}

void Sub_pcl(const PointCloud::ConstPtr& msg){
    queue <float> temp_x;
    queue <float> temp_y;
    queue <float> temp_z;
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
        temp_x.push(pt.x), temp_y.push(pt.y), temp_z.push(pt.z);
    }
    point_x = temp_x;
    point_y = temp_y;
    point_z = temp_z;
}

void Sub_odom(const nav_msgs::Odometry::ConstPtr& msg){
    position[0] = msg->pose.pose.position.x;
    position[1] = msg->pose.pose.position.y;
    position[2] = msg->pose.pose.position.z;
    position[3] = Quaternion2Euler(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void Pub_obj_points(queue <float> x,queue <float> y, queue <float> z,ros::Publisher pub_topic){
    PointCloudRGB::Ptr msg (new PointCloudRGB);
    msg->header.frame_id = "world";
    msg->height = x.size();
    msg->width = 1;
    while(!x.empty()){
        pcl::PointXYZRGB p;
        p.x = x.front();
        p.y = y.front();
        p.z = z.front()+lidar_height;
        p.r = 255;
        p.g = 255;
        p.b = 255;
        msg->points.push_back (p);
        x.pop();
        y.pop();
        z.pop();
    }
    if(point_x.size()==0){
        msg->points.push_back (pcl::PointXYZRGB(0.0, 0.0, 0.0));
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub_topic.publish (msg);
}

void Pub_simplified_points(queue <float> x,queue <float> y,ros::Publisher pub_topic){
    PointCloudRGB::Ptr msg (new PointCloudRGB);
    msg->header.frame_id = "world";
    msg->height = x.size();
    msg->width = 1;
    while(!x.empty()){
        pcl::PointXYZRGB p;
        p.x = x.front();
        p.y = y.front();
        p.z = 0;
        p.r = 255;
        p.g = 255;
        p.b = 0;
        msg->points.push_back (p);
        x.pop();
        y.pop();
    }
    if(point_x.size()==0){
        msg->points.push_back (pcl::PointXYZRGB(0.0, 0.0, 0.0));
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub_topic.publish (msg);
}

void Pub_pcl_test(queue <float> x,queue <float> y, queue <float> z,ros::Publisher pub_topic){
    PointCloudRGB::Ptr msg (new PointCloudRGB);
    msg->header.frame_id = "world";
    msg->height = x.size();
    msg->width = 1;
    while(!x.empty()){
        pcl::PointXYZRGB p;
        p.x = x.front();
        p.y = y.front();
        // p.z = z.front();
        p.z = z.front()+lidar_height;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        msg->points.push_back (p);
        x.pop();
        y.pop();
        z.pop();
    }
    if(point_x.size()==0){
        msg->points.push_back (pcl::PointXYZRGB(0.0, 0.0, 0.0));
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub_topic.publish (msg);
}

void Pub_a_pcl_test(double x,double y, double z,ros::Publisher pub_topic){
    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "world";
    msg->height = 1;
    msg->width = 1;
    msg->points.push_back (pcl::PointXYZ(x, y, z+lidar_height));
    if(point_x.size()==0){
        msg->points.push_back (pcl::PointXYZ(0.0, 0.0, 999.0));
    }
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub_topic.publish (msg);
}

void Pub_ground(double data_A,double data_B, double data_C, double data_D, ros::Publisher pub_topic){

    //  calculate orientation_x & orientation_y
    double ori_x = acos(data_B/sqrt(data_B*data_B+data_C*data_C))*180/M_PI;
    double ori_y = acos(data_A/sqrt(data_A*data_A+data_C*data_C))*180/M_PI;

    ori_x = ori_x-90;
    ori_y = 90-ori_y;

    if(-90.0<=ori_x<=90.0){
        ori_x = ori_x/90.0;
    }else if(ori_x<=-90.0){
        ori_x = (ori_x+180.0)/90.0;
    }else{
        ori_x = (ori_x-180)/90.0;
    }
    if(-90.0<=ori_y<=90.0){
        ori_y = ori_y/90.0;
    }else if(ori_y<=-90.0){
        ori_y = (ori_y+180.0)/90.0;
    }else{
        ori_y = (ori_y-180)/90.0;
    }

    double ww,rr,gg,bb,aa;
    if(data_A!=0 && data_B!=0 && data_C!=0 && data_D!=0){
        ww = detect_range*2;
        rr = 0.929;
        gg = 0.992;
        bb = 0.984;
        aa = 0.2;
    }else{
        ww = 0.001;
        rr = 0.001;
        gg = 0.001;
        bb = 0.001;
        aa = 0.001;
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "RANSAC_plane";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = (data_A*position[0] + data_B*position[1] + data_D)/(-data_C);
    marker.pose.orientation.x = ori_x;
    marker.pose.orientation.y = ori_y;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = ww;
    marker.scale.y = ww;
    marker.scale.z = 0.04;
    marker.color.a = aa;
    marker.color.r = rr;
    marker.color.g = gg;
    marker.color.b = bb;
    pub_topic.publish( marker);
}

void Pub_walls(queue <float> w_x,queue <float> w_y, queue <float> w_lenth, queue <float> w_height, queue <float> w_angle, ros::Publisher pub_topic){
    int queue_size = w_x.size();
    double in_x[queue_size];
    double in_y[queue_size];
    double in_l[queue_size];
    double in_h[queue_size];
    double in_a[queue_size];

    // data reshape
    for(int i=0; i<queue_size;i++){
        in_x[i] = w_x.front();
        in_y[i] = w_y.front();
        in_l[i] = w_lenth.front();
        in_h[i] = w_height.front();
        if(w_angle.front()<90){
            in_a[i] = w_angle.front();
        }else{
            in_a[i] = w_angle.front()-180;
        }
        w_x.pop();
        w_y.pop();
        w_lenth.pop();
        w_height.pop();
        w_angle.pop();
    }

    for(int i=0; i<max_wall_num; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "walls";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        if(i<queue_size){
            marker.pose.position.x = in_x[i];
            marker.pose.position.y = in_y[i];
            marker.pose.position.z = in_h[i]/2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = in_a[i]/90;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = in_l[i];
            marker.scale.y = Wall_error;
            marker.scale.z = in_h[i];
            marker.color.a = 0.8;
            marker.color.r = 0.8;
            marker.color.g = 0.678;
            marker.color.b = 1.0;
        }else{
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.0001;
            marker.scale.y = 0.0001;
            marker.scale.z = 0.0001;
            marker.color.a = 0.0001;
            marker.color.r = 0.0001;
            marker.color.g = 0.0001;
            marker.color.b = 0.0001;
        }
        pub_topic.publish( marker);
    }
}

void Pub_k_center(queue <float> x,queue <float> y, ros::Publisher pub_topic){

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "K_center";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;
    int queue_size = x.size();
    for(int i=0;i<queue_size;i++){
        p.x = x.front();
        p.y = y.front();
        p.z = 0;
        x.pop();
        y.pop();

        line_list.points.push_back(p);
        p.z += k_center_line_wifth;
        line_list.points.push_back(p);

    }
    pub_topic.publish(line_list);
}

void Pub_static_box(queue <float> x, queue <float> y ,queue <float> angle, queue <float> lenth, queue <float> width, queue <float> height, queue <float> next_v, ros::Publisher pub_topic){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "static_box";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.075;
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;
    int queue_size = x.size();
    int box_num = 0;
    for(int i=0;i<queue_size;i++){
        if(next_v.front()<static_to_unstable_error && x.front()!=float(99.9) && y.front()!=float(99.9)){
            box_num++;
            //button right
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button left
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button left front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);

            //top right
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button left
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button left front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);

            // right front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // right back
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // left front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // left back
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);
        }
        x.pop();
        y.pop();
        angle.pop();
        lenth.pop();
        width.pop();
        height.pop();
        next_v.pop();
    }
    if(!box_num){
        // pub something to topic
        line_list.color.a = 0.0001;
        p.x = 0; p.y = 0; p.z = 0;
        line_list.points.push_back(p);
        p.x = 0.0; p.y = 0.0; p.z = 0.0001;
        line_list.points.push_back(p);
    }
    pub_topic.publish(line_list);
}

void Pub_dynamic_box(queue <float> x, queue <float> y ,queue <float> angle, queue <float> lenth, queue <float> width, queue <float> height, queue <float> direction, queue <float> next_v,ros::Publisher pub_topic){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "static_box";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.075;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;
    int queue_size = x.size();
    int box_num = 0;
    for(int i=0;i<queue_size;i++){
        if(next_v.front()>static_to_unstable_error && x.front()!=float(99.9) && y.front()!=float(99.9)){
            box_num++;
            //button right
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button left
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button left front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);

            //top right
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button left
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button left front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            line_list.points.push_back(p);
            //button front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            line_list.points.push_back(p);

            // right front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()+width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // right back
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()+lenth.front()/2,y.front()-width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // left front
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()+width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // left back
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.z = 0;
            line_list.points.push_back(p);
            p.x = Box_rotate_x(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.y = Box_rotate_y(x.front(),y.front(),angle.front(),x.front()-lenth.front()/2,y.front()-width.front()/2);
            p.z = height.front();
            line_list.points.push_back(p);

            // arrow
            double arrow_body = 1.3;
            double arrow_length = 0.5;
            double arrow_width = 0.3;
            if(dynamic_max_speed>next_v.front() && next_v.front()>unstable_to_dynanic_error){
                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()*0+0.0,y.front()+0.0);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()*0+0.0,y.front()+0.0);
                p.z = height.front()/2;
                line_list.points.push_back(p);
                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front(),y.front()+0.0);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front(),y.front()+0.0);
                p.z = height.front()/2;
                line_list.points.push_back(p);

                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front(),y.front()+0.0);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front(),y.front()+0.0);
                p.z = height.front()/2;
                line_list.points.push_back(p);
                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()+arrow_width);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()+arrow_width);
                p.z = height.front()/2;
                line_list.points.push_back(p);

                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()+arrow_width);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()+arrow_width);
                p.z = height.front()/2;
                line_list.points.push_back(p);
                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()-arrow_width);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()-arrow_width);
                p.z = height.front()/2;
                line_list.points.push_back(p);

                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()-arrow_width);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front()-arrow_length,y.front()-arrow_width);
                p.z = height.front()/2;
                line_list.points.push_back(p);
                p.x = Box_rotate_x(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front(),y.front()+0.0);
                p.y = Box_rotate_y(x.front(),y.front(),direction.front(),x.front()+width.front()/2+next_v.front(),y.front()+0.0);
                p.z = height.front()/2;
                line_list.points.push_back(p);
            }
        }
        x.pop();
        y.pop();
        angle.pop();
        lenth.pop();
        width.pop();
        height.pop();
        direction.pop();
        next_v.pop();
    }
    if(!box_num){
        // pub something to topic
        line_list.color.a = 0.0001;
        p.x = 0; p.y = 0; p.z = 0;
        line_list.points.push_back(p);
        p.x = 0.0; p.y = 0.0; p.z = 0.0001;
        line_list.points.push_back(p);
    }
    pub_topic.publish(line_list);
}

void Remove_ground_and_walls(queue <float> x,queue <float> y, queue <float> z, queue <float>& out_x,queue <float>& out_y, queue <float>& out_z, double &out_A,double &out_B, double &out_C, double &out_D, queue <float>& wall_x, queue <float>& wall_y, queue <float>& wall_lenth, queue <float>& wall_height, queue <float>& wall_angle){
    int all_points_size = x.size();
    double best_plane_p1[3];
    double best_plane_p2[3];
    double best_plane_p3[3];
    int best_count;
    bool find_ground_flag = false;
    out_A = 0;
    out_B = 0;
    out_C = 0;
    out_D = 0;
    out_x = {};
    out_y = {};
    out_z = {};
    wall_x = {};
    wall_y = {};
    wall_lenth = {};
    wall_height = {};
    wall_angle = {};
    queue <float> temp_x = {};
    queue <float> temp_y = {};
    queue <float> temp_z = {};
    queue <float> register_x = {};
    queue <float> register_y = {};
    queue <float> register_z = {};

    // test_x_p = {};
    // test_y_p = {};
    // test_z_p = {};

    // remove data that out of range
    for(int i=0;i<all_points_size;i++){
        if(Cal_dis(x.front(),y.front(),0,position[0],position[1],0)<=detect_range){
            temp_x.push(x.front());
            temp_y.push(y.front());
            temp_z.push(z.front());
            x.pop();
            y.pop();
            z.pop();
        }else{
            x.pop();
            y.pop();
            z.pop();
        }
    }
    x = temp_x;
    y = temp_y;
    z = temp_z;

    // detect plane
    for(int qq=0;qq<max_wall_num;qq++){
        int queue_size = x.size();
        // cout<<"size = "<<x.size()<<endl;
        double in_x[x.size()];
        double in_y[y.size()];
        double in_z[z.size()];
        //data reshape
        for(int i=0;i<queue_size;i++){
            in_x[i] = x.front();
            in_y[i] = y.front();
            in_z[i] = z.front();
            x.pop();
            y.pop();
            z.pop();
        }

        // RANSAC
        best_count = 0;
        for(int w=0;w<Ransac_iter_times;w++){
            // pick three points
            clock_t time_now = clock();
            int random_num1 = int(time_now/(double)CLOCKS_PER_SEC*1000000);
            int p1 = random_num1%(queue_size+1);
            int random_num2 = int(time_now/(double)CLOCKS_PER_SEC*100000);
            int p2 = random_num1%(queue_size+1);
            int random_num3 = int(time_now/(double)CLOCKS_PER_SEC*10000);
            int p3 = random_num1%(queue_size+1);
            if(p1==p2 || p2==p3 || p1 == p3){
                p1 = rand()%(queue_size+1);
                p2 = rand()%(queue_size+1);
                p3 = rand()%(queue_size+1);
            }

            // three point to plane
            double x1 = in_x[p1];
            double y1 = in_y[p1];
            double z1 = in_z[p1];
            double x2 = in_x[p2];
            double y2 = in_y[p2];
            double z2 = in_z[p2];
            double x3 = in_x[p3];
            double y3 = in_y[p3];
            double z3 = in_z[p3];

            double A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
            double B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
            double C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
            double D = -(A * x1 + B * y1 + C * z1);

            // count points in range
            int count_in_range = 0;
            for(int i=0;i<queue_size;i++){
                double dis = fabs((A*in_x[i] + B*in_y[i] + C*in_z[i] + D ) / sqrt(A*A + B*B + C*C));
                if(dis<Ransac_range){
                    count_in_range = count_in_range + 1;
                }
            }

            if(count_in_range>best_count){
                best_plane_p1[0] = in_x[p1];
                best_plane_p1[1] = in_y[p1];
                best_plane_p1[2] = in_z[p1];
                best_plane_p2[0] = in_x[p2];
                best_plane_p2[1] = in_y[p2];
                best_plane_p2[2] = in_z[p2];
                best_plane_p3[0] = in_x[p3];
                best_plane_p3[1] = in_y[p3];
                best_plane_p3[2] = in_z[p3];
                best_count = count_in_range;
            }
        }

        // best plane parameters
        double best_A = (best_plane_p2[1] - best_plane_p1[1])*(best_plane_p3[2] - best_plane_p1[2]) - (best_plane_p2[2] - best_plane_p1[2])*(best_plane_p3[1] - best_plane_p1[1]);
        double best_B = (best_plane_p3[0] - best_plane_p1[0])*(best_plane_p2[2] - best_plane_p1[2]) - (best_plane_p2[0] - best_plane_p1[0])*(best_plane_p3[2] - best_plane_p1[2]);
        double best_C = (best_plane_p2[0] - best_plane_p1[0])*(best_plane_p3[1] - best_plane_p1[1]) - (best_plane_p3[0] - best_plane_p1[0])*(best_plane_p2[1] - best_plane_p1[1]);
        double best_D = -(best_A * best_plane_p1[0] + best_B * best_plane_p1[1] + best_C * best_plane_p1[2]);

        best_A = best_A/best_C;
        best_B = best_B/best_C;
        best_D = best_D/best_C;
        best_C = best_C/best_C;

        // cout<<"angle with ground = "<<angle_with_plane_and_plane(best_A,best_B,best_C,0,0,1)<<endl;
        // // cout<<point_to_plane_distance(out_A,out_B,out_C,out_D,position[0],position[1],0)<<endl;

        // calculate the density of the plane
        double wall_density = 0;
        double w_angle = 0;
        double wall_max_h = -999;
        double wall_min_h = 999;
        double wall_max_x = -999;
        double wall_min_x = 999;
        double wall_max_y = -999;
        double wall_min_y = 999;

        // calculate the angle of the wall
        w_angle = angle_with_plane_and_plane(best_A,best_B,0,0,1,0);

        // calculate the size of wall's area
        queue <float> wall_tf_x = {};
        queue <float> wall_tf_y = {};
        queue <float> wall_tf_z = {};
        for(int k=0;k<queue_size;k++){
            if(point_to_plane_distance(best_A, best_B, best_C, best_D, in_x[k], in_y[k], in_z[k])<Wall_error){
                // Project point to plane
                double project_x = project_point_to_plane(best_A,best_B,best_C,best_D, in_x[k], in_y[k], in_z[k], 0);
                double project_y = project_point_to_plane(best_A,best_B,best_C,best_D, in_x[k], in_y[k], in_z[k], 1);
                double project_z = project_point_to_plane(best_A,best_B,best_C,best_D, in_x[k], in_y[k], in_z[k], 2);

                if(project_z>wall_max_h){
                    wall_max_h = project_z;
                }
                if(wall_min_h>project_z){
                    wall_min_h = project_z;
                }
                if(project_x>wall_max_x){
                    wall_max_x = project_x;
                }
                if(wall_min_x>project_x){
                    wall_min_x = project_x;
                }
                if(project_y>wall_max_y){
                    wall_max_y = project_y;
                }
                if(wall_min_y>project_y){
                    wall_min_y = project_y;
                }
            }
        }

        // slice walls into pieces
        vector<vector<int> >slice_list(wall_slice_pieces/2);
        for(int i = 0; i <wall_slice_pieces/2; i++){
            slice_list[i].resize(wall_slice_pieces);
        }

        // calculate points density
        double wall_rt[3] = {wall_max_x,wall_min_y,wall_max_h};
        double wall_rb[3] = {wall_max_x,wall_min_y,wall_min_h};
        double wall_lt[3] = {wall_min_x,wall_max_y,wall_max_h};
        double wall_lb[3] = {wall_min_x,wall_max_y,wall_min_h};

        for(int k=0;k<queue_size;k++){
            if(point_to_plane_distance(best_A, best_B, best_C, best_D, in_x[k], in_y[k], in_z[k])<Wall_error){
                // Project point to new plane
                double x1 = wall_rt[0];
                double y1 = wall_rt[1];
                double z1 = wall_rt[2];
                double x2 = wall_lt[0];
                double y2 = wall_lt[1];
                double z2 = wall_lt[2];
                double x3 = wall_lb[0];
                double y3 = wall_lb[1];
                double z3 = wall_lb[2];
                double a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
                double b = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
                double c = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
                double d = -(a * x1 + b * y1 + c * z1);
                double project_x = project_point_to_plane(a, b, c, d, in_x[k], in_y[k], in_z[k], 0);
                double project_y = project_point_to_plane(a, b, c, d, in_x[k], in_y[k], in_z[k], 1);
                double project_z = project_point_to_plane(a, b, c, d, in_x[k], in_y[k], in_z[k], 2);

                // wall_tf_x.push(project_x);
                // wall_tf_y.push(project_y);
                // wall_tf_z.push(project_z);

                // bottom to top
                int b2t = -1;
                bool hit_flag = false;
                for(int i=0;i<wall_slice_pieces/2 && !hit_flag;i++){
                    if(wall_min_h+(wall_max_h-wall_min_h)/wall_slice_pieces*2*i<=project_z && project_z<=wall_min_h+(wall_max_h-wall_min_h)/wall_slice_pieces*2*(i+1)){
                        b2t = i;
                        hit_flag = true;
                    }
                }

                // left to right
                double aaa = wall_lb[0]-wall_lt[0];
                double bbb = wall_lb[1]-wall_lt[1];
                double ccc = wall_lb[2]-wall_lt[2];
                double ddd = -(aaa*wall_lt[0]+bbb*wall_lt[1]+ccc*wall_lt[2]);
                double project_xxx = project_point_to_plane(aaa, bbb, ccc, ddd, project_x, project_y, project_z, 0);
                double project_yyy = project_point_to_plane(aaa, bbb, ccc, ddd, project_x, project_y, project_z, 1);
                double project_zzz = project_point_to_plane(aaa, bbb, ccc, ddd, project_x, project_y, project_z, 2);

                int l2r = -1;
                hit_flag = false;
                for(int i=0;i<wall_slice_pieces && !hit_flag;i++){
                    if(wall_lt[0]+(wall_rt[0]-wall_lt[0])/wall_slice_pieces*i<=project_xxx && project_xxx<=wall_lt[0]+(wall_rt[0]-wall_lt[0])/wall_slice_pieces*(i+1)){
                        l2r = i;
                        hit_flag = true;
                    }
                }

                if(b2t != -1 && l2r != -1){
                    slice_list[b2t][l2r] = slice_list[b2t][l2r] + 1;
                }
            }
        }

        // show slice array
        // for(int l=0;l<wall_slice_pieces/2;l++){
        //     for(int q=0;q<wall_slice_pieces;q++){
        //         if(slice_list[l][q]>1){
        //             cout <<" "<<"0";
        //         }else{
        //             cout <<" "<<"_";
        //         }
        //     }
        //     cout <<endl;
        // }

        // calculate the proportion of wall points
        int sum = 0;
        for(int l=0;l<wall_slice_pieces/2;l++){
            for(int q=0;q<wall_slice_pieces;q++){
                if(slice_list[l][q]>1){
                    sum = sum + 1;
                }else{
                }
            }
        }

        wall_density = double(sum)/(wall_slice_pieces*wall_slice_pieces/2);
        // cout <<"wall_density = "<<wall_density<<endl;

        // cout<<angle_with_plane_and_plane(best_A,best_B,best_C,0,0,1)<<endl;
        // cout<<wall_density <<endl;
        // cout<<sqrt(pow(wall_max_x-wall_min_x,2)+pow(wall_max_y-wall_min_y,2))*abs(wall_max_h-wall_min_h)<<endl;

        if(angle_with_plane_and_plane(best_A,best_B,best_C,0,0,1)<Ground_plane_error && !find_ground_flag){
            // found ground and remove ground points
            // cout<<"ggggg"<<endl;
            out_A = best_A;
            out_B = best_B;
            out_C = best_C;
            out_D = best_D;
            // remove ground
            for(int k=0;k<queue_size;k++){
                if(in_z[k]<=Ground_error+(best_A*in_x[k]+best_B*in_y[k]+best_D)/(-best_C)){
                    in_x[k] = 99.9;
                    in_y[k] = 99.9;
                    in_z[k] = 99.9;
                    find_ground_flag = true;
                }
            }
        }else if(angle_with_plane_and_plane(best_A,best_B,best_C,0,0,1)>Wall_plane_error && wall_density >= wall_min_density && sqrt(pow(wall_max_x-wall_min_x,2)+pow(wall_max_y-wall_min_y,2))*abs(wall_max_h-wall_min_h)>Wall_min_size){
            // found wall and remove wall's points
            // cout<<"wwwww"<<endl;

            // remove points
            for(int k=0;k<queue_size;k++){
                if(point_to_plane_distance(best_A, best_B, best_C, best_D, in_x[k], in_y[k], in_z[k])<=Wall_error){
                    in_x[k] = 99.9;
                    in_y[k] = 99.9;
                    in_z[k] = 99.9;
                }
            }
            // push wall parameters
            wall_x.push((wall_max_x+wall_min_x)/2);
            wall_y.push((wall_max_y+wall_min_y)/2);
            wall_lenth.push(sqrt(pow(wall_max_x-wall_min_x,2)+pow(wall_max_y-wall_min_y,2)));
            wall_height.push(abs(wall_max_h-wall_min_h));
            // wall_angle.push(90+atan2((wall_max_y+wall_min_y)/2,(wall_max_x+wall_min_x)/2)*180/M_PI);
            wall_angle.push(w_angle);
        }else{
            // cout<<"no ground no wall"<<endl;
            for(int k=0;k<queue_size;k++){
                if(in_z[k]<=Wall_error+(best_A*in_x[k]+best_B*in_y[k]+best_D)/(-best_C)){
                    register_x.push(in_x[k]);
                    register_y.push(in_y[k]);
                    register_z.push(in_z[k]);
                    in_x[k] = 99.9;
                    in_y[k] = 99.9;
                    in_z[k] = 99.9;
                }
            }
        }

        // refill the points
        temp_x = {};
        temp_y = {};
        temp_z = {};
        for(int i=0;i<queue_size;i++){
            if(in_x[i]!= 99.9 && in_y[i]!= 99.9 && in_z[i]!= 99.9){
                temp_x.push(in_x[i]);
                temp_y.push(in_y[i]);
                temp_z.push(in_z[i]);
            }
        }
        x = temp_x;
        y = temp_y;
        z = temp_z;

        // if(qq==1){
        //     test_x_p = temp_x;
        //     test_y_p = temp_y;
        //     test_z_p = temp_z;
        // }
    }

    // output the points
    out_x = x;
    out_y = y;
    out_z = z;
    int refill_register = register_x.size();
    for(int i=0;i<refill_register;i++){
        out_x.push(register_x.front());
        out_y.push(register_y.front());
        out_z.push(register_z.front());
        register_x.pop();
        register_y.pop();
        register_z.pop();
    }
}

void Points_Simplification(queue <float> x,queue <float> y, queue <float>& out_x,queue <float>& out_y){
    double in_x[x.size()];
    double in_y[y.size()];
    int queue_size = x.size();
    vector<vector<int> >sim_list(point_sim_pieces);
	for(int i = 0; i <point_sim_pieces; i++){
		sim_list[i].resize(point_sim_pieces);
    }
    out_x = {};
    out_y = {};
    bool hit_flag = false;

    //data reshape
    for(int i=0;i<queue_size;i++){
        in_x[i] = x.front();
        in_y[i] = y.front();
        x.pop();
        y.pop();
    }

    // calculate points density
    for(int k=0;k<queue_size;k++){
        for(int i=0;i<point_sim_pieces&&!hit_flag;i++){
            for(int j=0;j<point_sim_pieces&&!hit_flag;j++){
                if(in_x[k]>position[0]-detect_range+detect_range*2/point_sim_pieces*j && position[0]-detect_range+detect_range*2/point_sim_pieces*(j+1)>in_x[k]){
                    if(in_y[k]<position[1]+detect_range-detect_range*2/point_sim_pieces*i && in_y[k]>position[1]+detect_range-detect_range*2/point_sim_pieces*(i+1)){
                        sim_list[i][j] = sim_list[i][j]+1;
                        hit_flag =- true;
                    }
                }
            }
        }
        hit_flag = false;
    }

    // from density to points
    for(int i=0;i<point_sim_pieces;i++){
        for(int j=0;j<point_sim_pieces;j++){
            if(sim_list[i][j]>=lattice_min_density){
                out_x.push(position[0]-detect_range+detect_range*2/point_sim_pieces*(j+0.5));
                out_y.push(position[1]+detect_range-detect_range*2/point_sim_pieces*(i+0.5));
            }
        }
    }

    // show simplified array
    // for(int l=0;l<point_sim_pieces;l++){
    //     for(int q=0;q<point_sim_pieces;q++){
    //         if(sim_list[l][q]){
    //             cout <<" "<<"0";
    //         }else{
    //             cout <<" "<<"_";
    //         }
    //     }
    //     cout <<endl;
    // }
}

void K_msans_plus_plus(queue <float> x,queue <float> y, queue <float>& out_x,queue <float>& out_y){
    out_x = {};
    out_y = {};
    int DCNT = x.size();

    // K-means param define
    #define DIM      2   /* 資料維度   */
    #define MAX_ITER 30  /* 最大迭代   */
    #define MIN_PT   0   /* 最小變動點 */
    int MAX_K = max_object_number;   /* 最大K值   */

    double sse2MD[MAX_K] = {};
    double sse_slope[MAX_K] = {};


    if (DCNT!=0){

        // K-means variable
        double data[DCNT][DIM]; /* 原始資料   */
        double cent[DCNT][DIM]; /* 重心       */
        int table[DCNT];        /* 資料所屬叢聚*/

        int ch_pt;             /* 紀錄變動之點 */
        int iter=0;            /* 迭代計數器   */
        double sse1;           /* 上一迭代之sse */
        double sse2;           /* 此次迭代之sse */

        // K-means++ variable
        double dps_table[DCNT][3] = {};
        int best_k;

        srand((unsigned)time(NULL));

        //  step 0 - 取得資料
        for(int i=0; i<DCNT; i++){
            data[i][0] = x.front();
            data[i][1] = y.front();
            x.pop();
            y.pop();
        }

        // #####################################################################
        // do kmeans++ for all K in MAX_K
        for(int K=0; K<MAX_K; K++){

            double dis_k[K][DIM];   /* 叢聚距離   */
            int cent_c[K];          /* 該叢聚資料數*/

            // k-means
            //step 1 - 初始化
            int i, j, k, rnd;
            int pick[K];
            for(i=0; i<K; i++){
                pick[i] = -1;
            }

            // // 隨機找K 個不同資料點
            // for(k=0; k<K; k++){
            //     rnd = rand() % DCNT; // 隨機取一筆
            //     for(i=0; i<k && pick[i]!=rnd; i++);
            //     if(i==k) pick[k]=rnd; // 沒重覆
            //     else --k; // 有重覆, 再找一次
            // }

            // 尋找K 使用K-means++優化
            for(i=0; i<K; i++){
                if(i==0){
                    pick[i] = rand() % DCNT;
                }else{// calculate dps_table
                    // calculate distance
                    int min_flag = -1;
                    for(j=0; j<DCNT; j++){
                        double distance_min = 99.9;
                        for(int k=0; k<i; k++){
                            if(Cal_dis(data[j][0],data[j][1],0,data[pick[k]][0],data[pick[k]][1],0)<distance_min){
                                distance_min = Cal_dis(data[j][0],data[j][1],0,data[pick[k]][0],data[pick[k]][1],0);
                            }
                        }
                        dps_table[j][0] = distance_min*distance_min;
                    }
                    // show dis table
                    double distance_max = -1;
                    int max_flag = -1;
                    for(j=0; j<DCNT; j++){
                        // cout << dps_table[j][0]<<" ";
                        if(dps_table[j][0]>distance_max){
                            distance_max = dps_table[j][0];
                            max_flag = j;
                        }
                    }
                    // cout << endl;
                    // cout << "far = " << temp_flag << " data = " << data[temp_flag][0] << " " << data[temp_flag][1] << endl;
                    pick[i] = max_flag;
                }
            }

            // show K
            // for(int i=0; i<K; i++){
            //     cout << pick[i] << " ";
            // }cout<<endl;

            // 將K 個資料點內容複制到重心cent
            for(k=0; k<K; k++){
                cent[k][0] = data[pick[k]][0];
                cent[k][1] = data[pick[k]][1];
            }

            /* step 2 - 更新一次對應表      */
            double dis, min_dis, t_sse=0.0;

            ch_pt=0;                          // 變動點數設0
            memset(cent_c, 0, sizeof(cent_c)); // 各叢聚資料數清0
            memset(dis_k, 0, sizeof(dis_k));   // 各叢聚距離和清0

            int min_k;
            for(i=0; i<DCNT; i++){
                // 尋找所屬重心
                min_dis = cal_sse(data[i], cent[0], DIM);
                min_k   = 0;
                for(k=1;k<K; k++){
                    dis = cal_sse(data[i], cent[k], DIM);
                    if(dis < min_dis){
                        min_dis=dis, min_k = k;
                    }
                }
                ch_pt+=(table[i]!=min_k); // 更新變動點數
                table[i] = min_k;          // 更新所屬重心
                ++cent_c[min_k];           // 累計重心資料數
                t_sse += min_dis;          // 累計總重心距離
                for(j=0; j<DIM; j++){
                    // 更新各叢聚總距離
                    dis_k[min_k][j]+=data[i][j];
                }
            }
            sse2 = t_sse;

            do{
                sse1 = sse2, ++iter;
                /* step 3 - 更新重心            */
                for(int k=0; k<K; k++){
                    for(int j=0; j<DIM; j++){
                        cent[k][j]=dis_k[k][j]/cent_c[k];
                    }
                }
                /* step 4 - 更新對應表          */
                double dis, min_dis, t_sse=0.0;

                ch_pt=0;                          // 變動點數設0
                memset(cent_c, 0, sizeof(cent_c)); // 各叢聚資料數清0
                memset(dis_k, 0, sizeof(dis_k));   // 各叢聚距離和清0

                for(i=0; i<DCNT; i++){
                    // 尋找所屬重心
                    min_dis = cal_sse(data[i], cent[0], DIM);
                    min_k   = 0;
                    for(k=1;k<K; ++k){
                        dis = cal_sse(data[i], cent[k], DIM);
                        if(dis < min_dis)
                            min_dis=dis, min_k = k;
                    }
                    ch_pt+=(table[i]!=min_k); // 更新變動點數
                    table[i] = min_k;          // 更新所屬重心
                    ++cent_c[min_k];           // 累計重心資料數
                    t_sse += min_dis;          // 累計總重心距離
                    for(j=0; j<DIM; j++){
                        // 更新各叢聚總距離
                        dis_k[min_k][j]+=data[i][j];
                    }
                }
                sse2 = t_sse;
            }while(iter<MAX_ITER && sse1!=sse2 && ch_pt>MIN_PT); // 收斂條件

            // 顯示最後重心位置
            // for(int k=0; k<K; k++) {
            //     out_x.push(cent[k][0]);
            //     out_y.push(cent[k][1]);
            //     // cout<< cent[k][0] << "  " << cent[k][1] << endl;
            // }
            // printf("sse   = %.2lf\n", sse2);
            // printf("ch_pt = %d\n", ch_pt);
            // printf("iter = %d\n", iter);
            // cout << "sse" << K << " = " << sse2 << endl;
            sse2MD[K] = sse2/K;
        }

        // #####################################################################
        // find best K

        // for(int i=0; i<MAX_K; i++) {
        //     cout << "MD " << i << " = " <<sse2MD[i] << endl;
        // }

        // calculate the angle of two line
        for(int i=0; i<MAX_K; i++) {
            if(i==0){
                sse_slope[i] = 0;
            }else if(i==MAX_K-1){
                sse_slope[i] = 0;
            }else{
                // two vector to angle
                double x_alpha = -1;
                double y_alpha = sse2MD[i-1]-sse2MD[i];
                double x_beta = 1;
                double y_beta = sse2MD[i+1]-sse2MD[i];
                sse_slope[i] = 180-acos((x_alpha*x_beta + y_alpha*y_beta)/sqrt((x_alpha*x_alpha + y_alpha*y_alpha)*(x_beta*x_beta+y_beta*y_beta)))*180 / M_PI;
            }
        }

        // for(int i=0; i<MAX_K; i++) {
        //     cout << "slope " << i << " = " <<sse_slope[i] << endl;
        // }

        // find max slpoe angle
        double max_slope_angle = -1;
        for(int i=0; i<MAX_K; i++) {
            // cout << "angle " << i << " = " <<sse_slope[i] << endl;
            if(sse_slope[i]>max_slope_angle){
                max_slope_angle = sse_slope[i];
                best_k = i;
            }
        }
        // cout<<"best k = "<<best_k<<endl;

        // #####################################################################
        // fianl K-means++
        int K = best_k;
        double dis_k[K][DIM];   /* 叢聚距離   */
        int cent_c[K];          /* 該叢聚資料數*/
        int i, j, k, rnd;
        int pick[K];
        for(i=0; i<K; i++){
            pick[i] = -1;
        }
        for(i=0; i<K; i++){
            if(i==0){
                pick[i] = rand() % DCNT;
            }else{
                int min_flag = -1;
                for(j=0; j<DCNT; j++){
                    double distance_min = 99.9;
                    for(int k=0; k<i; k++){
                        if(Cal_dis(data[j][0],data[j][1],0,data[pick[k]][0],data[pick[k]][1],0)<distance_min){
                            distance_min = Cal_dis(data[j][0],data[j][1],0,data[pick[k]][0],data[pick[k]][1],0);
                        }
                    }
                    dps_table[j][0] = distance_min*distance_min;
                }
                double distance_max = -1;
                int max_flag = -1;
                for(j=0; j<DCNT; j++){
                    // cout << dps_table[j][0]<<" ";
                    if(dps_table[j][0]>distance_max){
                        distance_max = dps_table[j][0];
                        max_flag = j;
                    }
                }
                pick[i] = max_flag;
            }
        }
        for(k=0; k<K; k++){
            cent[k][0] = data[pick[k]][0];
            cent[k][1] = data[pick[k]][1];
        }
        double dis, min_dis, t_sse=0.0;
        ch_pt=0;                          // 變動點數設0
        memset(cent_c, 0, sizeof(cent_c)); // 各叢聚資料數清0
        memset(dis_k, 0, sizeof(dis_k));   // 各叢聚距離和清0
        int min_k;
        for(i=0; i<DCNT; i++){
            min_dis = cal_sse(data[i], cent[0], DIM);
            min_k   = 0;
            for(k=1;k<K; k++){
                dis = cal_sse(data[i], cent[k], DIM);
                if(dis < min_dis){
                    min_dis=dis, min_k = k;
                }
            }
            ch_pt+=(table[i]!=min_k); // 更新變動點數
            table[i] = min_k;          // 更新所屬重心
            ++cent_c[min_k];           // 累計重心資料數
            t_sse += min_dis;          // 累計總重心距離
            for(j=0; j<DIM; j++){
                // 更新各叢聚總距離
                dis_k[min_k][j]+=data[i][j];
            }
        }
        sse2 = t_sse;

        do{
            sse1 = sse2, ++iter;
            for(int k=0; k<K; k++){
                for(int j=0; j<DIM; j++){
                    cent[k][j]=dis_k[k][j]/cent_c[k];
                }
            }
            double dis, min_dis, t_sse=0.0;

            ch_pt=0;                          // 變動點數設0
            memset(cent_c, 0, sizeof(cent_c)); // 各叢聚資料數清0
            memset(dis_k, 0, sizeof(dis_k));   // 各叢聚距離和清0

            for(i=0; i<DCNT; i++){
                min_dis = cal_sse(data[i], cent[0], DIM);
                min_k   = 0;
                for(k=1;k<K; ++k){
                    dis = cal_sse(data[i], cent[k], DIM);
                    if(dis < min_dis)
                        min_dis=dis, min_k = k;
                }
                ch_pt+=(table[i]!=min_k); // 更新變動點數
                table[i] = min_k;          // 更新所屬重心
                ++cent_c[min_k];           // 累計重心資料數
                t_sse += min_dis;          // 累計總重心距離
                for(j=0; j<DIM; j++){
                    dis_k[min_k][j]+=data[i][j];
                }
            }
            sse2 = t_sse;
        }while(iter<MAX_ITER && sse1!=sse2 && ch_pt>MIN_PT);
        for(int k=0; k<K; k++) {
            out_x.push(cent[k][0]);
            out_y.push(cent[k][1]);
        }
    }

    // special case when k=2 need to determine k=(1or2)
    if(out_x.size()==2){
        if(out_x.front()>=0 || out_x.front()<=0 || out_y.front()>=0 || out_y.front()<=0){
            // when k=1 => cent[][] = -nan
        }else{
            queue <float> temp_2_x = {};
            queue <float> temp_2_y = {};
            double temp_list[DCNT][2];
            double flag_x = 0;
            double flag_y = 0;
            double min_dis = 9999;
            double dis_sum = 0;

            // calculate k=1 center
            for(int i=0; i<DCNT; i++){
                temp_list[i][0] = x.front();
                temp_list[i][1] = y.front();
                x.pop();
                y.pop();
            }

            for(int j=0;j<DCNT;j++){
                dis_sum = 0;
                for(int i=0; i<DCNT; i++){
                    dis_sum += Cal_dis(temp_list[j][0],temp_list[j][1],0,temp_list[i][0],temp_list[i][1],0);
                }
                if(dis_sum<min_dis){
                    flag_x = temp_list[j][0];
                    flag_y = temp_list[j][1];
                }
            }

            temp_2_x.push(flag_x);
            temp_2_y.push(flag_y);
            out_x = temp_2_x;
            out_y = temp_2_y;
        }
    }

    // int wwwww = out_x.size();
    // for(int i=0;i<wwwww;i++){
    //     cout<<"x = "<<out_x.front()<<" y = "<<out_y.front()<<endl;
    //     out_x.pop();
    //     out_y.pop();
    // }

    // remove the points that too close
    if(out_x.size()>=2){
        queue <float> temp_3_x = {};
        queue <float> temp_3_y = {};
        double temp_xx[DCNT];
        double temp_yy[DCNT];
        int temp_size = out_x.size();
        for(int i=0; i<temp_size; i++){
            temp_xx[i] = out_x.front();
            temp_yy[i] = out_y.front();
            out_x.pop();
            out_y.pop();
        }

        // for(int i=0; i<temp_size; i++){
        //     cout<<temp_xx[i]<<" "<<temp_yy[i]<<endl;
        // }

        for(int i=0;i<temp_size; i++){
            for(int j=1;j<=temp_size;j++){
                if(i!=j && Cal_dis(temp_xx[i],temp_yy[i],0,temp_xx[j],temp_yy[j],0)<=big_item_error && temp_xx[i]!=99 && temp_xx[j]!=99){
                    temp_xx[i] = 1.0*(temp_xx[i]+temp_xx[j])/2.0;
                    temp_yy[i] = 1.0*(temp_yy[i]+temp_yy[j])/2.0;
                    temp_xx[j] = 99;
                    temp_yy[j] = 99;
                    // cout<<"i = "<<i<<" j = "<<j<<" "<<temp_xx[i]<<" "<<temp_yy[i]<<" "<<temp_xx[j]<<" "<<temp_yy[j]<<endl;
                }
            }
        }

        // cout<<"~~~"<<endl;
        // for(int i=0; i<temp_size; i++){
        //     cout<<temp_xx[i]<<" "<<temp_yy[i]<<endl;
        // }

        // refill output
        for(int i=0;i<temp_size; i++){
            if(temp_xx[i]!=99 && temp_yy[i]!=99){
                temp_3_x.push(temp_xx[i]);
                temp_3_y.push(temp_yy[i]);
            }
            // cout<<temp_xx[i]<<" "<<temp_yy[i]<<endl;
        }
        out_x = temp_3_x;
        out_y = temp_3_y;
    }
}

void Dynamic_tracking(queue <float> rrre_x, queue <float> rrre_y, queue <float> rre_x, queue <float> rre_y, queue <float> re_x,queue <float> re_y, queue <float> no_x,queue <float> no_y, queue <float>& o_dir, queue <float>& o_r, queue <float>& update_xxx, queue <float>& update_yyy, queue <float>& update_xx, queue <float>& update_yy, queue <float>& update_x, queue <float>& update_y){
    int be_be_before_size = rre_x.size();
    int be_before_size = rre_x.size();
    int before_size = re_x.size();
    int now_size = no_x.size();

    // make data lenth same
    while(before_size!=now_size){
        if(before_size>now_size){
            no_x.push(99.9);
            no_y.push(99.9);
            now_size++;
        }else{
            re_x.push(99.9);
            re_y.push(99.9);
            before_size++;
        }
    }
    while(before_size>be_before_size){
        rre_x.push(99.9);
        rre_y.push(99.9);
        be_before_size++;
    }
    while(be_before_size>be_be_before_size){
        rre_x.push(99.9);
        rre_y.push(99.9);
        be_be_before_size++;
    }

    // delete redundant data ????????
    // if(before_size){
    //     if(re_x.back()==float(99.9) && re_y.back()==float(99.9) && no_x.back()==float(99.9) && no_y.back()==float(99.9)){
    //         // re_x.pop();
    //         // re_y.pop();
    //         // no_x.pop();
    //         // no_y.pop();
    //         cout<<"  %%  "<<endl;
    //     }
    // }
    double be_be_before_x[before_size+1];
    double be_be_before_y[before_size+1];
    double be_before_x[before_size+1];
    double be_before_y[before_size+1];
    double before_x[before_size+1];
    double before_y[before_size+1];
    double now_x[now_size+1];
    double now_y[now_size+1];
    o_dir = {};
    o_r = {};
    update_x = {};
    update_y = {};
    update_xx = {};
    update_yy = {};
    update_xxx = {};
    update_yyy = {};

    //data reshape
    for(int i=1;i<=before_size;i++){
        be_be_before_x[i] = rrre_x.front();
        be_be_before_y[i] = rrre_y.front();
        be_before_x[i] = rre_x.front();
        be_before_y[i] = rre_y.front();
        before_x[i] = re_x.front();
        before_y[i] = re_y.front();
        now_x[i] = no_x.front();
        now_y[i] = no_y.front();
        rrre_x.pop();
        rrre_y.pop();
        rre_x.pop();
        rre_y.pop();
        re_x.pop();
        re_y.pop();
        no_x.pop();
        no_y.pop();
    }

    // // show raw data
    // cout<<"before :"<<endl;
    // for(int i=1;i<=before_size;i++){
    //     cout << before_x[i]<<" "<<before_y[i]<<endl;
    // }
    // cout<<"now :"<<endl;
    // for(int i=1;i<=now_size;i++){
    //     cout << now_x[i]<<" "<<now_y[i]<<endl;
    // }

    // Gale–Shapley matching
    int N, i, j, m, w;
	N = before_size;
    int Ranking[N+1][N+1], ManPref[N+1][N+1], WomanPref[N+1][N+1], Next[N+1], Current[N+1];

	double temp_w_x[N+1];
    double temp_w_y[N+1];
    double temp_m_x[N+1];
    double temp_m_y[N+1];

    // list of men who are not currently engaged
    queue <int> freeMen;

	// turn lacations to preference
	for(int k=1; k<=N; k++){
		// data copy
		for(int j=1; j<=N; j++){
			temp_w_x[j] = now_x[j];
			temp_w_y[j] = now_y[j];
			temp_m_x[j] = before_x[j];
			temp_m_y[j] = before_y[j];
		}

		for(int i=1; i<=N; i++){
			double min_w_dis = 999;
			double min_m_dis = 999;
			int min_w_flag = -1;
			int min_m_flag = -1;
			for(int j=1; j<=N; j++){
				if(Cal_dis(before_x[k],before_y[k],0,temp_w_x[j],temp_w_y[j],0)<min_w_dis){
					min_w_dis = Cal_dis(before_x[k],before_y[k],0,temp_w_x[j],temp_w_y[j],0);
					min_w_flag = j;
				}
				if(Cal_dis(now_x[k],now_y[k],0,temp_m_x[j],temp_m_y[j],0)<min_m_dis){
					min_m_dis = Cal_dis(now_x[k],now_y[k],0,temp_m_x[j],temp_m_y[j],0);
					min_m_flag = j;
				}
			}
			WomanPref[k][i] = min_w_flag;
			ManPref[k][i] = min_m_flag;
			temp_w_x[min_w_flag] = 999;
			temp_w_y[min_w_flag] = 999;
			temp_m_x[min_m_flag] = 999;
			temp_m_y[min_m_flag] = 999;
		}
	}

    // matching
	for (i = 1; i <= N; i++){
		for (j = 1; j <= N; j++){
			Ranking[i][WomanPref[i][j]] = j;
        }
    }

	memset(Current, 0, (N + 1) * sizeof(int));

	for (i = 1; i <= N; i++) {
		freeMen.push(i);
		Next[i] = 1;
	}

	while (!freeMen.empty())    {
		m = freeMen.front();
		// cout << "Next"<<Next[m]<<endl;
		w = ManPref[m][Next[m]++];
		// cout <<"man"<< m<<endl;
		// cout << "weman"<<w<<endl;
		if (Current[w] == 0)   {
			// cout<<"=="<<endl;
			Current[w] = m;
			freeMen.pop();
		} else if (Ranking[w][m] < Ranking[w][Current[w]])  {
			// cout<<"wwwww"<<endl;
			freeMen.pop();
			freeMen.push(Current[w]);
			Current[w] = m;
		}
	}

	// // show stable matching
	// for (w = 1; w <= N; w++){
	// 	printf("%d %d\n", Current[w], w);
    // }

    // now data update
    for(int i=1;i<=N;i++){
        update_x.push(now_x[Current[i]]);
        update_y.push(now_y[Current[i]]);
    }

    // before data update
    for(int i=1;i<=before_size;i++){
        update_xx.push(before_x[i]);
        update_yy.push(before_y[i]);
    }

    // before before data update
    for(int i=1;i<=before_size;i++){
        update_xxx.push(be_before_x[i]);
        update_yyy.push(be_before_y[i]);
    }

    // // show all data
    // for(int i=1; i<=N; i++){
    //     cout<<"bebefo = "<<be_before_x[i]<<"  "<< be_before_y[i]<<endl;
    //     cout<<"before = "<<before_x[i]<<"  "<< before_y[i]<<endl;
    //     cout<<"now    = "<<now_x[Current[i]]<<"  "<< now_y[Current[i]]<<endl;
    // }

    // calculate direction
    for(int i=1; i<=N; i++){
        double xx1 = before_x[i] - be_before_x[i];
        double yy1 = before_y[i] - be_before_y[i];
        double xx2 = now_x[Current[i]] - be_before_x[i];
        double yy2 = now_y[Current[i]] - be_before_y[i];
        double temp1 = atan2(yy1,xx1)*180/M_PI;
        double temp2 = atan2(yy2,xx2)*180/M_PI;
        if(be_before_x[i]!=float(99.9) && be_before_y[i]!=float(99.9)){
            if(temp1*temp2<0 && (temp1<-90 || temp2<-90)){
                o_dir.push((temp1+temp2+360)/2);
            }else{
                o_dir.push((temp1+temp2)/2);
            }
        }else{
            o_dir.push(temp1);
        }
    }

    // calculate distance
    for(int i=1; i<=N; i++){
        if(Cal_dis(now_x[Current[i]], now_y[Current[i]], 0, before_x[i], before_y[i], 0)*ros_hz > same_item_error || now_x[Current[i]]==float(99.9) || now_y[Current[i]]==float(99.9) || before_x[i]==float(99.9) || before_y[i]==float(99.9) || be_before_x[i]==float(99.9) || be_before_y[i]==float(99.9)|| be_be_before_x[i]==float(99.9) || be_be_before_y[i]==float(99.9)){
            o_r.push((static_to_unstable_error+unstable_to_dynanic_error)/2);
        }else{
            o_r.push((Cal_dis(now_x[Current[i]], now_y[Current[i]], 0, before_x[i], before_y[i], 0)+Cal_dis(now_x[Current[i]], now_y[Current[i]], 0, be_before_x[i], be_before_y[i], 0)+Cal_dis(before_x[i], before_y[i], 0, be_before_x[i], be_before_y[i], 0)+Cal_dis(be_before_x[i], be_before_y[i], 0, be_be_before_x[i], be_be_before_y[i], 0)+Cal_dis(before_x[i], before_y[i], 0, be_be_before_x[i], be_be_before_y[i], 0)+Cal_dis(now_x[Current[i]], now_y[Current[i]], 0, be_be_before_x[i], be_be_before_y[i], 0))/10*ros_hz);
        }
    }
}

void Bounding_box(queue <float> i_x,queue <float> i_y, queue <float> i_z, queue <float> k_x, queue <float> k_y,queue <float> o_x, queue <float> o_y, queue <float>& o_angle, queue <float>& o_length, queue <float>& o_width, queue <float>& o_height){

    int points_size = i_x.size();
    int k_size = k_x.size();
    double in_x[i_x.size()];
    double in_y[i_x.size()];
    double in_z[i_x.size()];
    int points_belongs[i_x.size()] = {};
    double in_k_x[k_x.size()];
    double in_k_y[k_x.size()];
    double k_dis_sum[k_x.size()] = {};
    double k_num_sum[k_x.size()] = {};
    o_x = {};
    o_y = {};
    o_angle = {};
    o_length = {};
    o_width = {};
    o_height = {};

    if(points_size!=0){
        //data reshape
        for(int i=0;i<points_size;i++){
            in_x[i] = i_x.front();
            in_y[i] = i_y.front();
            in_z[i] = i_z.front();
            i_x.pop();
            i_y.pop();
            i_z.pop();
        }
        for(int i=0; i<k_size; i++){
            in_k_x[i] = k_x.front();
            in_k_y[i] = k_y.front();
            k_x.pop();
            k_y.pop();
        }

        // show all k center
        // for(int i=0; i<k_size; i++){
        //     cout << in_k_x[i]<<" "<<in_k_y[i]<<endl;
        // }

        // classify points to each k canter
        for(int i=0; i<points_size; i++){
            double min_dis = 99;
            int min_flag = -1;
            for(int j=0; j<k_size; j++){
                if(Cal_dis(in_x[i], in_y[i], 0, in_k_x[j], in_k_y[j], 0) < min_dis){
                    min_dis = Cal_dis(in_x[i], in_y[i], 0, in_k_x[j], in_k_y[j], 0);
                    min_flag = j;
                }
            }
            points_belongs[i] = min_flag;
            k_dis_sum[min_flag] += Cal_dis(in_x[i], in_y[i], 0, in_k_x[min_flag], in_k_y[min_flag], 0);
            k_num_sum[min_flag] += 1;
        }

        // show points belongs
        // for(int j=0; j<k_size; j++){
        //     cout<<"k_x = "<<in_k_x[j]<<" k_y = "<<in_k_y[j]<<endl;
        //     for(int i=0; i<points_size; i++){
        //         if(points_belongs[i]==j){
        //             cout << in_x[i] <<" "<<in_y[i]<<" "<<points_belongs[i]<<endl;
        //         }
        //     }
        // }

        //show all k distance, k sum
        // for(int i=0;i<k_size;i++){
        //     cout<<k_dis_sum[i]<< " " << k_num_sum[i]<<endl;
        //     cout<<k_dis_sum[i]/k_num_sum[i]<<endl;
        // }

        // remove outliers
        for(int i=0; i<points_size; i++){
            for(int j=0; j<k_size; j++){
                if(Cal_dis(in_x[i], in_y[i], 0, in_k_x[points_belongs[i]], in_k_y[points_belongs[i]], 0) > 1.8*k_dis_sum[points_belongs[i]]/k_num_sum[points_belongs[i]]){
                    points_belongs[i] = 9;
                }
            }
        }

        // show points belongs after remove
        // for(int j=0; j<k_size; j++){
        //     cout<<"k_x = "<<in_k_x[j]<<" k_y = "<<in_k_y[j]<<endl;
        //     for(int i=0; i<points_size; i++){
        //         if(points_belongs[i]==j){
        //             cout << in_x[i] <<" "<<in_y[i]<<" "<<points_belongs[i]<<endl;
        //         }
        //     }
        // }

        // rotating cliper
        for(int i=0; i<k_size; i++){
            double find_top = -99;
            double best_max = 99;
            double best_min = 0;
            double min_angle = -1;
            // cout << "center = "<< in_k_x[i]<<"  "<< in_k_y[i]<<endl;
            // find x and z
            for(int j=0; j<180/rotate_angle_gap; j++){
                // cout <<"angle = "<<j*rotate_angle_gap<<endl;
                double x_max = -99;
                double x_min = 99;
                for(int k=0; k<points_size; k++){
                    if(points_belongs[k]==i){
                        if(Box_rotate_x(in_k_x[i],in_k_y[i],j*rotate_angle_gap,in_x[k],in_y[k])>x_max){
                            x_max = Box_rotate_x(in_k_x[i],in_k_y[i],j*rotate_angle_gap,in_x[k],in_y[k]);
                        }
                        if(x_min>Box_rotate_x(in_k_x[i],in_k_y[i],j*rotate_angle_gap,in_x[k],in_y[k])){
                            x_min = Box_rotate_x(in_k_x[i],in_k_y[i],j*rotate_angle_gap,in_x[k],in_y[k]);
                        }
                        if(in_z[k]>find_top){
                            find_top = in_z[k];
                        }
                    }
                }
                if(best_max-best_min>x_max-x_min){
                    best_max = x_max;
                    best_min = x_min;
                    min_angle = 90-j*rotate_angle_gap;
                }
                // cout << "max = "<<x_max<<"  "<<"min = "<< x_min<< "max-min = "<<x_max-x_min<<endl;
            }
            // cout << "best max = "<<best_max<<"  "<<"best min = "<< best_min<<"max-min = "<<best_max-best_min<<endl;
            // cout<< "best angle = "<<min_angle<<endl;
            o_x.push((best_max+best_min)/2);
            o_angle.push(min_angle);
            o_width.push(best_max-best_min+box_error_endure);

            // find y
            double y_max = -99;
            double y_min = 99;
            for(int j=0;j<points_size;j++){
                if(points_belongs[j]==i){
                    if(y_max<Box_rotate_y(in_k_x[i],in_k_y[i],-min_angle+90,in_x[j],in_y[j])){
                        y_max = Box_rotate_y(in_k_x[i],in_k_y[i],-min_angle+90,in_x[j],in_y[j]);
                    }
                    if(y_min>Box_rotate_y(in_k_x[i],in_k_y[i],-min_angle+90,in_x[j],in_y[j])){
                        y_min = Box_rotate_y(in_k_x[i],in_k_y[i],-min_angle+90,in_x[j],in_y[j]);
                    }
                }
            }
            o_y.push((y_max+y_min)/2);
            o_length.push(y_max-y_min+box_error_endure);
            o_height.push(find_top+lidar_height+box_error_endure);
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub = nh.subscribe<PointCloud>("/velodyne_points_tf", 1, Sub_pcl);
    ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom_new", 1000, Sub_odom);
    ros::Publisher obj_points_pub = nh.advertise<PointCloudRGB> ("/object_points", 1);
    ros::Publisher sim_points_pub = nh.advertise<PointCloudRGB> ("/simplified_points", 1);
    ros::Publisher K_center_pub = nh.advertise<visualization_msgs::Marker>("K_center", 10);
    ros::Publisher static_box_pub = nh.advertise<visualization_msgs::Marker>("static_box", 10);
    ros::Publisher dynamic_box_pub = nh.advertise<visualization_msgs::Marker>("dynamic_box", 10);
    ros::Publisher test_points_pub = nh.advertise<PointCloudRGB> ("/test_points", 1);
    ros::Publisher a_points_pub = nh.advertise<PointCloud> ("/a_points", 1);
    ros::Publisher plane_cylinder_pub = nh.advertise<visualization_msgs::Marker>( "/ground_plane", 0);
    ros::Publisher walls_cuble_pub = nh.advertise<visualization_msgs::Marker>( "/walls_plane", 0);
    ros::Rate loop_rate(ros_hz);
    while (ros::ok()){
        clock_t time_start = clock();
        double ground_A,ground_B,ground_C,ground_D;
        queue <float> obj_x, obj_y, obj_z;
        // queue <float> with_walls_x, with_walls_y, with_walls_z;
        queue <float> walls_x, walls_y, walls_lenth, walls_height, walls_angle;
        queue <float> sim_x, sim_y;
        queue <float> k_means_x, k_means_y;
        queue <float> box_angle, box_length, box_width, box_height;
        queue <float> next_dir, next_r;
        cout << "---\n";
        ////
        Remove_ground_and_walls(point_x, point_y, point_z, obj_x, obj_y, obj_z, ground_A, ground_B, ground_C, ground_D, walls_x, walls_y, walls_lenth, walls_height, walls_angle);
        Pub_ground(ground_A, ground_B, ground_C, ground_D, plane_cylinder_pub);
        Pub_walls(walls_x, walls_y, walls_lenth, walls_height, walls_angle, walls_cuble_pub);
        Pub_obj_points(obj_x, obj_y, obj_z, obj_points_pub);

        Pub_pcl_test(test_x_p,test_y_p,test_z_p,test_points_pub);

        Points_Simplification(obj_x, obj_y, sim_x, sim_y);
        Pub_simplified_points(sim_x, sim_y, sim_points_pub);

        K_msans_plus_plus(sim_x, sim_y, k_means_x, k_means_y);

        Dynamic_tracking(place_record_3_x, place_record_3_y,place_record_2_x, place_record_2_y, place_record_1_x, place_record_1_y, k_means_x, k_means_y, next_dir, next_r, place_record_3_x, place_record_3_y,place_record_2_x, place_record_2_y, place_record_1_x, place_record_1_y);

        Bounding_box(obj_x,obj_y,obj_z, place_record_1_x,place_record_1_y, place_record_1_x, place_record_1_y, box_angle,box_length,box_width,box_height);
        Pub_k_center(place_record_1_x, place_record_1_y, K_center_pub);
        Pub_static_box(place_record_1_x, place_record_1_y, box_angle, box_length, box_width, box_height, next_r, static_box_pub);
        Pub_dynamic_box(place_record_1_x, place_record_1_y, box_angle, box_length, box_width, box_height, next_dir, next_r, dynamic_box_pub);

        cout << "raw = " <<point_x.size() << " obj = " <<obj_x.size() << " sim = " << sim_x.size() << " K = " << k_means_x.size() << endl;
        ////
        clock_t time_end = clock();
        cout << "used " << (time_end - time_start)/(double)CLOCKS_PER_SEC << " ms\n";
        ros::spinOnce ();
        loop_rate.sleep ();
    }
    ros::spin();
}
