#include<iostream>
#include<fstream>
#include<eigen3/Eigen/Eigen>
#include<ros/ros.h>
#include<tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ofstream foutC;
geometry_msgs::PoseStamped local_pose;

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pose = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "save_traj_as_tum");
    ros::NodeHandle nh;
    
    foutC.open("/home/hsj/test1.txt");
    ros::Subscriber local_pos_pub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,local_pose_cb);
    ros::Rate rate(30);
    while(ros::ok()){
        foutC << local_pose.header.stamp.nsec << " ";

        ROS_INFO("%f %f %f %f %f %f %f",local_pose.pose.position.x,local_pose.pose.position.y,local_pose.pose.position.z,local_pose.pose.orientation.x,local_pose.pose.orientation.y,local_pose.pose.orientation.z,local_pose.pose.orientation.w);
        foutC << local_pose.pose.position.x <<" " << local_pose.pose.position.y << " " << local_pose.pose.position.z << " " << local_pose.pose.orientation.x << " " << local_pose.pose.orientation.y << " " << local_pose.pose.orientation.z << " " << local_pose.pose.orientation.w << std::endl;
        rate.sleep();
        ros::spinOnce();
    }
    foutC.close();
    return 0;
}
