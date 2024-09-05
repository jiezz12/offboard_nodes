#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

bool marker_found = false, flag_move = false, flag_land = false;
int32_t current_target_id = 0;
float detec_x = 0, detec_y = 0, detec_z = 0;

apriltag_ros::AprilTagDetection marker;
void apriltag_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    int count = msg->detections.size();
	if(count!=0)
	{
		for(int i = 0; i<count; i++)
		{
			marker = msg->detections[i];			
		}
        detec_x = marker.pose.pose.pose.position.x;
        detec_y = marker.pose.pose.pose.position.y;
        detec_z = marker.pose.pose.pose.position.z; 

	ROS_INFO("detec_x=%.2f,detec_y=%.2f,detec_z=%.2f",detec_x,detec_y,detec_z);
	
	}

}

int main(int argc, char *argv[])
{

	ros::init(argc,argv,"detection");
	ros::NodeHandle nh;

//订阅识别信息
ros::Subscriber yolov5_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 10, apriltag_cb);
//发布yolo标志位

	ros::Rate loop_rate(20);
    ROS_INFO("begin");
	while(ros::ok())
    	{


	        ros::spinOnce();//订阅话题应用
	        loop_rate.sleep();
	}
}

