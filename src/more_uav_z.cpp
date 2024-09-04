#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>  
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

double a1, b1, c1 , a2, b2;
double roll,pitch,yaw = 0, target_yaw = 0, diff_angle = 0, err_yaw = 0,err_yaw0 = 0, err_yaw_err = 0;
int flag_circle = 0;
#define HIGH    2  //起飞高度
#define RATE            10  // 频率 hz
#define RADIUS          10   // 绕圆运动的半径大小 m
#define CYCLE_S         10 // 完成一次绕圆运动所花费的时间
#define STEPS           (CYCLE_S*RATE)
#define VEL_Z		0.03	//Z轴速度

typedef struct
{	
	float kp = 0.88;              //比例系数
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

//uav0所有消息类型
mavros_msgs::State current_state_uav0;
mavros_msgs::PositionTarget setpoint_uav0; 
mavros_msgs::PositionTarget path0[STEPS];
geometry_msgs::TwistStamped vel_msg_uav0;
geometry_msgs::PoseStamped local_pos_uav0;

//uav1所有消息类型
mavros_msgs::State current_state_uav1;
mavros_msgs::PositionTarget setpoint_uav1; 
mavros_msgs::PositionTarget path1[STEPS];
geometry_msgs::TwistStamped vel_msg_uav1;
geometry_msgs::PoseStamped local_pos_uav1;

void state_cb0(const mavros_msgs::State::ConstPtr& msg){
    current_state_uav0 = *msg;
}
void local_pos_cb0(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos_uav0 = *msg;
}
void  vel_cb0(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	vel_msg_uav0 = *msg;
}

void state_cb1(const mavros_msgs::State::ConstPtr& msg){
    current_state_uav1 = *msg;
}
void local_pos_cb1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos_uav1 = *msg;
}
void  vel_cb1(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	vel_msg_uav1 = *msg;
}

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

    //ROS_INFO("roll: %.0f ,pitch: %.0f  , yaw: %.0f",roll,pitch,yaw);  
}

void init_step()
{
	int i;
   	const double r = RADIUS;

    for(i=0;i<STEPS;i++)
    {
        path0[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        path0[i].type_mask = 0;

        
        double a = i*(2*M_PI/STEPS);
        double c = cos(a);
        double s = sin(a);

        path0[i].position.x =  -r * c ; 
        path0[i].position.y =  -r * s;
        path0[i].position.z =  i * VEL_Z + c1;

	path1[i].position.x =  a1 ; 
        path1[i].position.y =  b1 ;
	path1[i].position.z =  i * VEL_Z + c1;
/*
        path[i].velocity.x =  -r*s ;
        path[i].velocity.z= r*c;
        path[i].velocity.y = vel_y;

        path[i].acceleration_or_force.x =  -r*c;
        path[i].acceleration_or_force.z = -r*s;
        path[i].acceleration_or_force.y = 0.00;
*/
	path0[i].yaw =target_yaw*M_PI/180; 
	}
}

int main(int argc, char **argv)
{
setlocale(LC_ALL, "");

ros::init(argc, argv, "offb_node"); //ros初始化，最后一个参数为节点名称
ros::NodeHandle nh;

//uav0所有话题及服务
ros::Subscriber state_sub_uav0 = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, state_cb0);
ros::Subscriber local_pos_sub_uav0 = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 10, local_pos_cb0);
ros::Subscriber vel_sub_uav0 = nh.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/local_position/velocity_local", 10, vel_cb0);
ros::Subscriber yaw_sub_uav0 = nh.subscribe<sensor_msgs::Imu>("/uav0/mavros/imu/data", 10, yaw_cb);
ros::Publisher setpoint_pub_uav0 = nh.advertise<mavros_msgs::PositionTarget>("/uav0/mavros/setpoint_raw/local", 10);
ros::ServiceClient arming_client_uav0 = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
ros::ServiceClient set_mode_client_uav0 = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");


//uav1所有话题及服务
ros::Subscriber state_sub_uav1 = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_cb1);
ros::Subscriber local_pos_sub_uav1 = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, local_pos_cb1);
ros::Subscriber vel_sub_uav1 = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 10, vel_cb1);
//ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("/uav0/mavros/imu/data", 10, yaw_cb);
ros::Publisher setpoint_pub_uav1 = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);
ros::ServiceClient arming_client_uav1 = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
ros::ServiceClient set_mode_client_uav1 = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");


ros::Rate rate(20.0);
 
while(ros::ok() && !current_state_uav0.connected && !current_state_uav1.connected){
    ros::spinOnce();
    rate.sleep();
}
setpoint_uav0.header.stamp = ros::Time::now();	//存储ROS中的时间戳信息
setpoint_uav0.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
setpoint_uav0.type_mask =			//使用位置控制
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
mavros_msgs::PositionTarget::IGNORE_YAW ;
//mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

setpoint_uav1.header.stamp = ros::Time::now();	//存储ROS中的时间戳信息
setpoint_uav1.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
setpoint_uav1.type_mask =			//使用位置控制
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
mavros_msgs::PositionTarget::IGNORE_YAW ;
//mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

setpoint_uav0.position.x = 0;
setpoint_uav0.position.y = 0;
setpoint_uav0.position.z = 0;

setpoint_uav1.position.x = 0;
setpoint_uav1.position.y = 0;
setpoint_uav1.position.z = 0;

for(int i = 100; ros::ok() && i > 0; --i){
	setpoint_pub_uav0.publish(setpoint_uav0);
	setpoint_pub_uav1.publish(setpoint_uav1);
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

//ROS_INFO("UAV 启动!");
int step = 0;
int sametimes = 0;
target_yaw = yaw; 
a1 = local_pos_uav0.pose.position.x;
b1 = local_pos_uav0.pose.position.y;
a2 = local_pos_uav1.pose.position.x;
b2 = local_pos_uav1.pose.position.y;
c1 = 2;
init_step();
int i = 0;

while(ros::ok())//进入大循环
{
  if (current_state_uav0.mode != "OFFBOARD" && current_state_uav1.mode != "OFFBOARD"  && (ros::Time::now() - last_request > ros::Duration(5.0)))
	{
		if (set_mode_client_uav0.call(offb_setPS_mode) && set_mode_client_uav1.call(offb_setPS_mode) && offb_setPS_mode.response.mode_sent)
		{
			ROS_INFO("POSTION PROTECTED");
		}
		last_request = ros::Time::now();
	}
	else
	{
        if( !current_state_uav0.armed && !current_state_uav1.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client_uav0.call(arm_cmd) && arming_client_uav1.call(arm_cmd) && arm_cmd.response.success)
            {
		ROS_INFO("UAV0,UAV1 启动!");               
            }
            last_request = ros::Time::now();
        }
	else
	{
		switch(step)
		{
			case 0:
				setpoint_uav0.position.x = a1;
				setpoint_uav0.position.y = b1;
				setpoint_uav0.position.z = c1;

				setpoint_uav1.position.x = a2;
				setpoint_uav1.position.y = b2;
				setpoint_uav1.position.z = c1;
				if (local_pos_uav0.pose.position.z > c1- 0.1 && local_pos_uav0.pose.position.z < c1 + 0.1 && local_pos_uav1.pose.position.z > c1- 0.1 && local_pos_uav1.pose.position.z < c1 + 0.1)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 1;
						ROS_INFO("case:1");
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
				err_yaw = target_yaw - yaw;//计算偏航角速度
				err_yaw_err =err_yaw - err_yaw0;
 				err_yaw0 = err_yaw;
				diff_angle = H.kp * err_yaw + H.kd * err_yaw_err;
				setpoint_uav0.yaw_rate = diff_angle * 0.01;
   				setpoint_pub_uav0.publish(setpoint_uav0);
				setpoint_pub_uav1.publish(setpoint_uav1);
	 	                break;
			case 1:
				
				setpoint_pub_uav0.publish(path0[i]);
				setpoint_pub_uav1.publish(path1[i]);
				i++;
				if(i >= STEPS)
				{
					i = 0;
					flag_circle++;
					ROS_INFO("圈数:%d圈",flag_circle);
					c1 = local_pos_uav0.pose.position.z;
					init_step();
				}
				if (flag_circle > 10)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 2;
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
	 	                break;
			case 2:
				setpoint_uav0.position.x = a1;
				setpoint_uav0.position.y = b1;
				setpoint_uav0.position.z = 2;

				setpoint_uav1.position.x = a2;
				setpoint_uav1.position.y = b2;
				setpoint_uav1.position.z = 2;

				if (local_pos_uav0.pose.position.x > a1 - 0.1 && local_pos_uav0.pose.position.x < a1 + 0.1 && local_pos_uav0.pose.position.y > b1 - 0.1 && local_pos_uav0.pose.position.y < b1 + 0.1 && local_pos_uav0.pose.position.z < 2.1 && local_pos_uav0.pose.position.z > 1.9)
                  		{
					if (local_pos_uav1.pose.position.x > a2 - 0.1 && local_pos_uav1.pose.position.x < a2 + 0.1 && local_pos_uav1.pose.position.y > b2 - 0.1 && local_pos_uav1.pose.position.y < b2 + 0.1 && local_pos_uav1.pose.position.z < 2.1 && local_pos_uav1.pose.position.z > 1.9)
					{
     		  	                if (sametimes > 20)
              				{
         		                   step = 3;
						ROS_INFO("case:3");
                        		}
                        		else
                           		sametimes++;
                    			}
					}
                    			else sametimes = 0;
				err_yaw = target_yaw - yaw;
				err_yaw_err =err_yaw - err_yaw0;
 				err_yaw0 = err_yaw;
				diff_angle = H.kp * err_yaw + H.kd * err_yaw_err;
				setpoint_uav0.yaw_rate = diff_angle * 0.01;
   				setpoint_pub_uav0.publish(setpoint_uav0);
				setpoint_pub_uav1.publish(setpoint_uav1);
	 	                break;
			case 3:
				offb_set_mode.request.custom_mode = "AUTO.LAND";
				if (current_state_uav0.mode != "AUTO.LAND" && current_state_uav1.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
				{
					if (set_mode_client_uav0.call(offb_set_mode) && set_mode_client_uav1.call(offb_set_mode) && offb_set_mode.response.mode_sent)
					{
						ROS_INFO("AUTO.LAND enabled");
					}
					last_request = ros::Time::now();
					}
				break;
				default:
					break;
		}
	}
    } 

	 
	

    ros::spinOnce();
    rate.sleep();
}

return 0;

}
