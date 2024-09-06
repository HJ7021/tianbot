#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
using namespace std;
int flag = 0;
ofstream foutC1;
void odomCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if(flag == 0)
    {
        for(int i=0;i<msg->poses.size();i++)
        {
            foutC1 << msg->poses[i].pose.position.x<<" " 
                   << msg->poses[i].pose.position.y<<" "
                   << msg->poses[i].pose.orientation.w << std::endl;
        }
    }
    foutC1.close();
    ROS_INFO("finish");
    flag=1;
}

int main(int argc, char **argv) {
    foutC1.open("/home/hj/tianbot_ws/src/tianracer/tianracer_navigation/path/raicom_2.txt");
    ros::init(argc, argv, "path");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub_ = nh.subscribe<nav_msgs::Path> ("/tianracer/move_base/TebLocalPlannerROS/global_plan",1,odomCallback);
    ros::spin();
}