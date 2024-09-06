#ifndef SRC_PURE_PURSUIT_H
#define SRC_PURE_PURSUIT_H

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "math.h"

class PurePursuit {
public:
    // 构造函数与析构函数
    PurePursuit();

    ~PurePursuit();

    void initMarker();
    bool crossesLine(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end, const geometry_msgs::Point& robot_position);

    void PurePursuitCallback(const std_msgs::Int32::ConstPtr& msg);

    void callback(const sensor_msgs::LaserScan& lidar_info);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

    void pathCallback(const nav_msgs::Path::ConstPtr &pathMsg);

    void controlLoopCallback(const ros::TimerEvent&);

    double getEta(const geometry_msgs::Pose& carPose);

    double getSteering(double eta);

    double PointDistanceSquare(geometry_msgs::PoseStamped& wayPt,const geometry_msgs::Point& car_pos);
    int minIndex(const geometry_msgs::Point& carPose);

    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);

    bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);

    bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);

    ros::NodeHandle nh_;    // 创建公共的节点句柄
private:
    ros::Subscriber odom_sub_, path_sub_, goal_sub_,PurePursuit_sub_,scan_sub;
    ros::Publisher ackermann_pub_, marker_pub_, plan_pub_,cmd_vel_pub;
    ros::Timer timer1, timer2;
    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_;
    visualization_msgs::Marker points_, line_strip_, goal_circle_;
    geometry_msgs::Point odom_goal_pos_, goal_pos_;
    geometry_msgs::Twist cmd_vel;
    sensor_msgs::LaserScan lidar_info;
    ackermann_msgs::AckermannDriveStamped ackermann_cmd_;
    nav_msgs::Odometry odom_;
    nav_msgs::Path map_path_, odom_path_;
    geometry_msgs::Point line_start_;
    geometry_msgs::Point line_end_;
    int Pure_Pursuit_out; 
    int lap_count_;  //记圈


    std::string  global_plan;


    double base_shape_L, Lfw, reference_v, velocity, base_angle_,steering_,max_speed,min_speed,middle_speed;
    double steering_gain, goal_radius, speed_incremental;
    double perpendicular_tolerance, v1, v2, v3, kz, ka;  //横向容忍程度perpendicular_tolerance
    int controller_freq_,lap_count_max;
    bool foundForwardPt_, goal_received_, goal_reached_, debug_mode;
};

#endif //SRC_PURE_PURSUIT_H
