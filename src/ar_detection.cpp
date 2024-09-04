#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>  
#include <mavros_msgs/PositionTarget.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#define HIGHT	1.1		//初始飞行高度
#define MIN_ERROR 0.03		//最小像素差

//using namespace std;

bool marker_found = false;
int current_target_id = 0;
float init_x_take_off =0, init_y_take_off =0, init_z_take_off =0;
float detec_x = 0, detec_y = 0, detec_z = 0;
float altitude = 0;

typedef struct
{	
	float kp = 0.80;              //比例系数
	float ki = 0.0002;              //积分系数
	float kd = 1.68;              //微分系数
	
	float err_I_lim = 500;		//积分限幅值
	
	float errx_Now,errx_old_Last,errx_old_LLast;           //当前偏差,上一次偏差,上上次偏差
	float erry_Now,erry_old_Last,erry_old_LLast;
	float errz_Now,errz_old_Last,errz_old_LLast;
	float errax_Now,errax_old_Last,errax_old_LLast;           //当前偏差,上一次偏差,上上次偏差
	float erray_Now,erray_old_Last,erray_old_LLast;
	float erraz_Now,erraz_old_Last,erraz_old_LLast;
	

	float errx_p,errx_i,errx_d;
	float erry_p,erry_i,erry_d;
	float errz_p,errz_i,errz_d;
	float errax_p,errax_i,errax_d;
	float erray_p,erray_i,erray_d;
	float erraz_p,erraz_i,erraz_d;		
	
	float CtrOutx,CtrOuty,CtrOutz;          //控制增量输出
 	float CtrOutax,CtrOutay,CtrOutaz; 

        float OUTLIM = 0;		//输出限幅
	
}PID;
PID H;

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint; // 位置速度控制消息类
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}
//订阅检测二维码坐标
ar_track_alvar_msgs::AlvarMarker marker;	
void ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	int count = msg->markers.size();
	if(count!=0)
	{
		for(int i = 0; i<count; i++)
		{
			marker = msg->markers[i];			
			if(marker.id == current_target_id)
			{
				marker_found = true;
			}
		}
        detec_x = marker.pose.pose.position.x;
        detec_y = marker.pose.pose.position.y;
        detec_z = marker.pose.pose.position.z; 

	//ROS_INFO("detec_x=%.2f,detec_y=%.2f,detec_z=%.2f",detec_x,detec_y,detec_z);
	
	}
	else
	{
		marker_found = false;
	}
}

void vel_pid(float pose_x,float pose_y)
{
	H.errx_Now = 0 - pose_x;
	H.errx_p = H.errx_Now;
	H.errx_i = H.errx_Now + H.errx_i;
	H.errx_d = H.errx_Now - H.errx_old_Last + H.errx_old_Last - H.errx_old_LLast;
	H.erry_Now = 0- pose_y;
	H.erry_p = H.erry_Now;
	H.erry_i = H.erry_Now + H.erry_i;
	H.erry_d = H.erry_Now - H.erry_old_Last + H.erry_old_Last - H.erry_old_LLast;

	H.errx_old_LLast = H.errx_old_Last;
	H.errx_old_Last = H.errx_Now;
	H.erry_old_LLast = H.erry_old_Last;
	H.erry_old_Last = H.erry_Now;	
	
	//积分限幅
	if(H.errx_i > H.err_I_lim)	H.errx_i = H.err_I_lim;		
	if(H.errx_i < -H.err_I_lim)	H.errx_i = -H.err_I_lim;
	if(H.erry_i > H.err_I_lim)	H.erry_i = H.err_I_lim;		
	if(H.erry_i < -H.err_I_lim)	H.erry_i = -H.err_I_lim;

	H.CtrOutx = H.errx_p*H.kp + H.errx_i*H.ki + H.errx_d*H.kd;
	H.CtrOuty = H.erry_p*H.kp + H.erry_i*H.ki + H.erry_d*H.kd;

	H.OUTLIM = 1;
	if(H.OUTLIM != 0)
	{
		if(H.CtrOutx > H.OUTLIM)		H.CtrOutx = H.OUTLIM;
		if(H.CtrOutx < -H.OUTLIM)		H.CtrOutx = -H.OUTLIM;
		if(H.CtrOuty > H.OUTLIM)		H.CtrOuty = H.OUTLIM;
		if(H.CtrOuty < -H.OUTLIM)		H.CtrOuty = -H.OUTLIM;
	}	
	ROS_INFO("vel_x:%.2f,vel_y=%.2f",H.CtrOutx,H.CtrOuty);
}

int main(int argc, char **argv)
{
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "offb_node"); //ros初始化，最后一个参数为节点名称
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);//订阅无人机状态话题
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);//订阅位置信息
	ros::Subscriber ar_pos_sub = nh.subscribe("/ar_pose_marker", 10, ar_marker_cb);//订阅相机识别二维码相机坐标
	ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);//控制话题,可以发布位置速度加速度同时控制
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Rate rate(20.0);
 
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
setpoint.header.stamp = ros::Time::now();
setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
setpoint.type_mask =			//使用位置控制
//mavros_msgs::PositionTarget::IGNORE_PX |
//mavros_msgs::PositionTarget::IGNORE_PY |
//mavros_msgs::PositionTarget::IGNORE_PZ |
mavros_msgs::PositionTarget::IGNORE_VX |
mavros_msgs::PositionTarget::IGNORE_VY |
mavros_msgs::PositionTarget::IGNORE_VZ |
mavros_msgs::PositionTarget::IGNORE_AFX |
mavros_msgs::PositionTarget::IGNORE_AFY |
mavros_msgs::PositionTarget::IGNORE_AFZ |
mavros_msgs::PositionTarget::FORCE |
mavros_msgs::PositionTarget::IGNORE_YAW |
mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
setpoint.position.x = 0;
setpoint.position.y = 0;
setpoint.position.z = 0;

 for(int i = 100; ros::ok() && i > 0; --i){
        setpoint.header.stamp = ros::Time::now();
   	setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }


mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";

//设定无人机保护模式 POSTION
mavros_msgs::SetMode offb_setPS_mode;
offb_setPS_mode.request.custom_mode = "POSCTL";

mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;
 
ros::Time last_request = ros::Time::now();

init_x_take_off = local_pos.pose.position.x;
init_y_take_off = local_pos.pose.position.y;
init_z_take_off = local_pos.pose.position.z;

float a = 0, b = 0, c = 0;
char mode = 't';
int sametimes = 0;

	while(ros::ok()){

	if (current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0)))
		{
			if (set_mode_client.call(offb_setPS_mode) &&
				offb_setPS_mode.response.mode_sent)
			{
				ROS_INFO("POSTION PROTECTED");
			}
			last_request = ros::Time::now();
		}
		else
		{
			if (!current_state.armed &&
				(ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if (arming_client.call(arm_cmd) &&
					arm_cmd.response.success)
				{
					ROS_INFO("UAV armed");
					init_x_take_off = local_pos.pose.position.x;
					init_y_take_off = local_pos.pose.position.y;
					init_z_take_off = local_pos.pose.position.z;
				}
				last_request = ros::Time::now();
			}
		else
		{
		switch(mode)	
			{
			case 't':
				setpoint.position.x = init_x_take_off;
  				setpoint.position.y = init_y_take_off;
				setpoint.position.z = init_z_take_off + HIGHT;
				if(local_pos.pose.position.z > init_z_take_off + HIGHT - 0.1 && local_pos.pose.position.z < init_z_take_off + HIGHT + 0.1)
				{
					if (sametimes > 20)
              				{
         		                   mode = 'm';
						last_request = ros::Time::now();
                    			}
                    			else sametimes++;
                		}
				else sametimes = 0;
	 	                break;
			case 'm':
				setpoint.position.x = init_x_take_off + 3;
  				setpoint.position.y = init_y_take_off;
				setpoint.position.z = init_z_take_off + HIGHT;
				if(marker_found)
				{
					if (sametimes > 10)
              				{
         		                mode = 'l';
					a = local_pos.pose.position.x;
					b = local_pos.pose.position.y;
					altitude = local_pos.pose.position.z;
                        	}
                        	else sametimes++;
                		}
				else sametimes = 0;
				 if( ros::Time::now() - last_request > ros::Duration(20.0))
				{
					mode = 'j';
				}
				break;
			case 'l':
				setpoint.type_mask =
					mavros_msgs::PositionTarget::IGNORE_PX |
        				mavros_msgs::PositionTarget::IGNORE_PY |
        				mavros_msgs::PositionTarget::IGNORE_PZ |
        				mavros_msgs::PositionTarget::IGNORE_AFX |
        				mavros_msgs::PositionTarget::IGNORE_AFY |
        				mavros_msgs::PositionTarget::IGNORE_AFZ |
        				mavros_msgs::PositionTarget::FORCE |
        				mavros_msgs::PositionTarget::IGNORE_YAW;
				vel_pid(detec_x,detec_y);
				setpoint.velocity.x = H.CtrOuty;
				setpoint.velocity.y = H.CtrOutx;
				setpoint.velocity.z = -0.1;
				if(!marker_found)
				{
						setpoint.velocity.z = 0;
				}
				if(abs(detec_x) < MIN_ERROR && abs(detec_y) < MIN_ERROR)
				{
					setpoint.velocity.x = 0;
					setpoint.velocity.y = 0;
					/*
					if(local_pos.pose.position.z < init_z_take_off + 0.2)
					{
						if(sametimes > 20)
						{
						a = local_pos.pose.position.x;
						b = local_pos.pose.position.y;
						mode = 'k';
						}
						else sametimes++;
					}else sametimes = 0;
					*/
				}
				if(local_pos.pose.position.z < init_z_take_off + 0.4)
				{

					a = local_pos.pose.position.x;
					b = local_pos.pose.position.y;
					mode = 'j';
				}
				break;
			case 'j':
				offb_set_mode.request.custom_mode = "AUTO.LAND";
				if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
				{
					if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					{
						ROS_INFO("AUTO.LAND enabled");
					}
					last_request = ros::Time::now();
				}
				break;
			case 'k':
				setpoint.type_mask =			//使用位置控制
//mavros_msgs::PositionTarget::IGNORE_PX |
//mavros_msgs::PositionTarget::IGNORE_PY |
//mavros_msgs::PositionTarget::IGNORE_PZ |
mavros_msgs::PositionTarget::IGNORE_VX |
mavros_msgs::PositionTarget::IGNORE_VY |
mavros_msgs::PositionTarget::IGNORE_VZ |
mavros_msgs::PositionTarget::IGNORE_AFX |
mavros_msgs::PositionTarget::IGNORE_AFY |
mavros_msgs::PositionTarget::IGNORE_AFZ |
mavros_msgs::PositionTarget::FORCE |
mavros_msgs::PositionTarget::IGNORE_YAW |
mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
				setpoint.position.x = a;
  				setpoint.position.y = b;
				setpoint.position.z = init_z_take_off - 1;	
			default:
				break;
			}	
		}

	}

	setpoint_pub.publish(setpoint);

    ros::spinOnce();
    rate.sleep();
}

return 0;

}
