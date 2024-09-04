#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

#define FLIGHT_ALTITUDE 2.0f
#define RATE            15  // 频率 hz
#define RADIUS          8     // 绕八运动的半径大小 m
#define CYCLE_S         40  // 完成一次绕八运动所花费的时间
#define STEPS           (CYCLE_S*RATE)

#define PI 3.141592653589793238327950

double xhome = 0, yhome = 0, zhome = 0;
double roll,pitch,yaw = 0,target_yaw = 0;
int home_flag = 0;

mavros_msgs::State current_state;
mavros_msgs::PositionTarget path[STEPS];
geometry_msgs::PoseStamped local_pos;

void yaw_cb(sensor_msgs::Imu msg)
{
	tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
        );

	
    tf::Matrix3x3(quaternion).getRPY(roll,pitch,yaw);
    roll = roll*180/M_PI;
    pitch = pitch*180/M_PI;
    yaw = yaw*180/M_PI;  
}

void init_path()
{
    int i;
    const double dt = 1.0/RATE;
    const double dadt = (2.0*PI)/CYCLE_S;   //角度相对于时间的一阶导数
    const double r = RADIUS;

    for(i=0;i<STEPS;i++)
    {
        path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        path[i].type_mask = 0;

        
        double a = (-PI/2.0) + i*(2.0*PI/STEPS);
        double c = cos(a);
        double c2a = cos(2.0*a);
        double c4a = cos(4.0*a);
        double c2am3 = c2a - 3.0;
        double s = sin(a);
        double cc = c*c;
        double ss = s*s;
        double sspo = (s*s)+1.0;
        double ssmo = (s*s)-1.0;
        double sspos = sspo * sspo;

        path[i].position.x = (r*c) / sspo;
        path[i].position.y = -(r*c*s) / sspo;
        path[i].position.z = FLIGHT_ALTITUDE;

        path[i].velocity.x = -dadt*r*s*( ss + 2.0f*cc + 1.0f) / sspos;
        path[i].velocity.y = dadt*r*( ss*ss + ss + ssmo*cc) /sspos;
        path[i].velocity.z = 0;

        path[i].acceleration_or_force.x = -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/(c2am3*c2am3*c2am3);
        path[i].acceleration_or_force.y = dadt*dadt*r*((44.0*c2a) + c4a - 21.0)/(c2am3*c2am3*c2am3);
        path[i].acceleration_or_force.z = 0.0;

        //path[i].yaw = atan2(-path[i].velocity.x,path[i].velocity.y) + (PI/2.0f);
	path[i].yaw = target_yaw*M_PI/180;
        printf("x:%7.3f y:%7.3f yaw:%7.1f\n",path[i].position.x,path[i].position.y,path[i].yaw*180.0f/PI);

    }
    for(i=0;i<STEPS;i++){
        double next = path[(i+1)%STEPS].yaw;
        double curr = path[i].yaw;
        if((next-curr) < -PI) next+=(2.0*PI);
        if((next-curr) > PI) next-=(2.0*PI);
        path[i].yaw_rate = (next-curr)/dt;
    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
local_pos = *msg;
}


int main(int argc, char **argv)
{
    int i;

    ros::init(argc,argv,"figure_8_circle_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, yaw_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher target_local_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",10);

    ros::Rate rate(RATE);

    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCU...");
    }

    mavros_msgs::PositionTarget position_home;
    position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    position_home.position.x = 0;
    position_home.position.y = 0;
    position_home.position.z = FLIGHT_ALTITUDE;
    position_home.velocity.x = 0;
    position_home.velocity.y = 0;
    position_home.velocity.z = 0;
    position_home.acceleration_or_force.x = 0;
    position_home.acceleration_or_force.y = 0;
    position_home.acceleration_or_force.z = 0;
    position_home.yaw = (-45.0f + 90.0f) * PI / 180.0f;
    position_home.yaw_rate = 0;

   

    for(i=100;ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

	ros::Time last_request = ros::Time::now();
	mavros_msgs::SetMode offb_setPS_mode;
	offb_setPS_mode.request.custom_mode = "POSCTL";
	mavros_msgs::CommandBool arm_cmd;
   	arm_cmd.request.value = true;

HOME:
    ROS_INFO("waiting for offboard mode");
    while(ros::ok()){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
       if(current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
		if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
			{
				if (arming_client.call(arm_cmd) &&
					arm_cmd.response.success)
				{
           			 	if(home_flag==1)
           				 {            
           				 	xhome = local_pos.pose.position.x;
          				  	yhome = local_pos.pose.position.y;
         				   	zhome = local_pos.pose.position.z;
          				  	home_flag=0;
						target_yaw = yaw;
		    				ROS_INFO("target_yaw=%.2f",target_yaw);
           				 }break;		
				}
			}
	}
    }
    

    i = RATE * 2;
    ROS_INFO("going home");
    while(ros::ok() && i>0){
        if(current_state.mode != "OFFBOARD" ){
            //goto HOME;
		if (set_mode_client.call(offb_setPS_mode) &&
				offb_setPS_mode.response.mode_sent)
			{
				ROS_INFO("POSTION PROTECTED");
				
			}
        }
        i--;
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }
	 
	init_path();
    i=0;
    ROS_INFO("following path");
    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            //goto HOME;
	if (set_mode_client.call(offb_setPS_mode) &&
				offb_setPS_mode.response.mode_sent)
			{
				ROS_INFO("POSTION PROTECTED");
			}
        }
        target_local_pub.publish(path[i]);
        i++;
	ROS_INFO("yaw=%.2f",yaw);
        if(i>=STEPS) i=0;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
