#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  



geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}

Vec3d camera_to_world (const Vec3d& p_c)
{   
    geometry_msgs::PointStamped camera;
    geometry_msgs::PointStamped world;
    camera.header.frame_id = camera_frame;
    
    camera.point.x = p_c[0];
    camera.point.y = p_c[1];
    camera.point.z = p_c[2];

    if (!tfListener_->waitForTransform(map_frame, camera_frame, ros::Time(0), ros::Duration(3)))
    {
        ROS_ERROR("Could not get transform from %s to %s after 1 second!", map_frame.c_str(), camera_frame.c_str());
    }
    tfListener_->transformPoint(map_frame,camera,world);
    //cout<<"world"<<world.point.x << ","<<world.point.y << ","<<world.point.z << endl;
    return Vec3d(world.point.x,world.point.y,current_pose.pose.position.z);
}



int main(int argc, char **argv)
{
ros::init(argc, argv, "pix2world"); //ros初始化，最后一个参数为节点名称
ros::NodeHandle nh;

ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);//订阅位置信息


}