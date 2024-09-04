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

double a1, b1, c1;
double roll,pitch,yaw = 0, target_yaw = 0, diff_angle = 0, err_yaw = 0,err_yaw0 = 0, err_yaw_err = 0;
int flag_circle = 0;
#define HIGH    2  //起飞高度
#define RATE            20  // 频率 hz
#define RADIUS          8   // 绕圆运动的半径大小 m
#define CYCLE_S         40 // 完成一次绕圆运动所花费的时间
#define STEPS           (CYCLE_S*RATE)
#define VEL_Z		0.01	//y轴速度

mavros_msgs::State current_state;
mavros_msgs::PositionTarget setpoint; // 位置速度控制消息类
mavros_msgs::PositionTarget path[STEPS];
geometry_msgs::TwistStamped vel_msg;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}
void  vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
	vel_msg = *msg;
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
        path[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        path[i].type_mask = 0;

        if(i<=STEPS/2)
	{
        double a = i*(2*M_PI/STEPS*2);
        double c = cos(a + M_PI);
        double s = sin(a + M_PI);
	
        path[i].position.x =  r + r * c; 
        path[i].position.y =  r * s;
        path[i].position.z =  HIGH;
	}
	else
	{
	double a = i*(2*M_PI/STEPS*2);
        double c = cos(a);
        double s = sin(a);
	path[i].position.x =  -r + r * c; 
        path[i].position.y =  -r * s;
        path[i].position.z =  HIGH;
	}
/*
        path[i].velocity.x =  -r*s ;
        path[i].velocity.z= r*c;
        path[i].velocity.y = vel_y;

        path[i].acceleration_or_force.x =  -r*c;
        path[i].acceleration_or_force.z = -r*s;
        path[i].acceleration_or_force.y = 0.00;
*/
	path[i].yaw =target_yaw*M_PI/180; 

	}
}

void fuction_init()
{
	
}
int main(int argc, char **argv)
{
setlocale(LC_ALL, "");

ros::init(argc, argv, "offb_node"); //ros初始化，最后一个参数为节点名称
ros::NodeHandle nh;


//订阅。<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为10）、回调函数
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);//订阅位置信息
ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, vel_cb);//订阅速度信息
ros::Subscriber yaw_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, yaw_cb);//订阅无人机imu数据

ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);//控制话题,可以发布位置速度加速度同时控制

ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
 
ros::Rate rate(20.0);
 
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
setpoint.header.stamp = ros::Time::now();	//存储ROS中的时间戳信息
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
mavros_msgs::PositionTarget::IGNORE_YAW ;
//mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
setpoint.position.x = 0;
setpoint.position.y = 0;
setpoint.position.z = 0;

for(int i = 100; ros::ok() && i > 0; --i){
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

//ROS_INFO("UAV 启动!");
int step = 0;
int sametimes = 0;
target_yaw = yaw; 
a1 = local_pos.pose.position.x;
b1 = local_pos.pose.position.y;
c1 = 2;
init_step();
int i = 0;

while(ros::ok())//进入大循环
{
  if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
	{
		if (set_mode_client.call(offb_setPS_mode) && offb_setPS_mode.response.mode_sent)
		{
			ROS_INFO("POSTION PROTECTED");
		}
		last_request = ros::Time::now();
	}
	else
	{
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
		ROS_INFO("UAV 启动!");               
            }
            last_request = ros::Time::now();
        }
	else
	{
		switch(step)
		{
			case 0:
				setpoint.position.x = 0;
				setpoint.position.y = 0;
				setpoint.position.z = 2;
				if (local_pos.pose.position.z > 1.9 && local_pos.pose.position.z < 2.1)
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
				setpoint.yaw_rate = diff_angle * 0.01;
   				setpoint_pub.publish(setpoint);
	 	                break;
			case 1:
				
				setpoint_pub.publish(path[i]);
				printf("x:%f,y:%f\n",path[i].position.x,path[i].position.y);
				i++;
				if(i >= STEPS)
				{
					i = 0;
					flag_circle++;
					ROS_INFO("圈数:%d圈",flag_circle);
					c1 = local_pos.pose.position.z;
					//init_step();
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
				setpoint.position.x = 0;
				setpoint.position.y = 0;
				setpoint.position.z = 2;
				if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1 && local_pos.pose.position.y > -0.1 && local_pos.pose.position.y < 0.1 && local_pos.pose.position.z < 2.1 && local_pos.pose.position.z > 1.9)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 3;
						ROS_INFO("case:3");
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
				err_yaw = target_yaw - yaw;
				err_yaw_err =err_yaw - err_yaw0;
 				err_yaw0 = err_yaw;
				diff_angle = H.kp * err_yaw + H.kd * err_yaw_err;
				setpoint.yaw_rate = diff_angle * 0.01;
   				setpoint_pub.publish(setpoint);
	 	                break;
			case 3:
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
