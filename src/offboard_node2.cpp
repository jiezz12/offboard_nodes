#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>	//添加速度话题消息类型的头文件

typedef struct
{	
	float kp = 0.88;              //比例系数
	float ki = 0.0002;              //积分系数
	float kd = 1.68;              //微分系数
	
	float err_I_lim = 500;		//积分限幅值
	
	float errx_Now,errx_old_Last,errx_old_LLast;           //当前偏差,上一次偏差,上上次偏差
	float erry_Now,erry_old_Last,erry_old_LLast;
	float errz_Now,errz_old_Last,errz_old_LLast;
	
	float errx_p,errx_i,errx_d;
	float erry_p,erry_i,erry_d;
	float errz_p,errz_i,errz_d;		
	
	float CtrOutx,CtrOuty,CtrOutz;          //控制增量输出
 
        float OUTLIM = 0;		//输出限幅
	
}PID;
PID H;

geometry_msgs::Twist vector;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 订阅的无人机当前位置数据
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	local_pos = *msg;
}

void vec_pid(float pose_x,float pose_y,float pose_z,float limt)
{
	H.errx_Now = pose_x - local_pos.pose.position.x;
	H.errx_p = H.errx_Now;
	H.errx_i = H.errx_Now + H.errx_i;
	H.errx_d = H.errx_Now - H.errx_old_Last + H.errx_old_Last - H.errx_old_LLast;
	H.erry_Now = pose_y - local_pos.pose.position.y;
	H.erry_p = H.erry_Now;
	H.erry_i = H.erry_Now + H.erry_i;
	H.erry_d = H.erry_Now - H.erry_old_Last + H.erry_old_Last - H.erry_old_LLast;
	H.errz_Now = pose_z - local_pos.pose.position.z;
	H.errz_p = H.errz_Now;
	H.errz_i = H.errz_Now + H.errz_i;
	H.errz_d = H.errz_Now - H.errz_old_Last + H.errz_old_Last - H.errz_old_LLast;

	H.errx_old_LLast = H.errx_old_Last;
	H.errx_old_Last = H.errx_Now;
	H.erry_old_LLast = H.erry_old_Last;
	H.erry_old_Last = H.erry_Now;	
	H.errz_old_LLast = H.errz_old_Last;
	H.errz_old_Last = H.errz_Now;	
	
	//积分限幅
	if(H.errx_i > H.err_I_lim)	H.errx_i = H.err_I_lim;		
	if(H.errx_i < -H.err_I_lim)	H.errx_i = -H.err_I_lim;
	if(H.erry_i > H.err_I_lim)	H.erry_i = H.err_I_lim;		
	if(H.erry_i < -H.err_I_lim)	H.erry_i = -H.err_I_lim;
	if(H.errz_i > H.err_I_lim)	H.errz_i = H.err_I_lim;		
	if(H.errz_i < -H.err_I_lim)	H.errz_i = -H.err_I_lim;

	H.CtrOutx = H.errx_p*H.kp + H.errx_i*H.ki + H.errx_d*H.kd;
	H.CtrOuty = H.erry_p*H.kp + H.erry_i*H.ki + H.erry_d*H.kd;
	H.CtrOutz = H.errz_p*H.kp + H.errz_i*H.ki + H.errz_d*H.kd;	

	
	H.OUTLIM = limt;
	if(H.OUTLIM != 0)
	{
		if(H.CtrOutx > H.OUTLIM)		H.CtrOutx = H.OUTLIM;
		if(H.CtrOutx < -H.OUTLIM)		H.CtrOutx = -H.OUTLIM;
		if(H.CtrOuty > H.OUTLIM)		H.CtrOuty = H.OUTLIM;
		if(H.CtrOuty < -H.OUTLIM)		H.CtrOuty = -H.OUTLIM;
	}	
	
 
	vector.linear.x = H.CtrOutx;
	vector.linear.y = H.CtrOuty;
	vector.linear.z = H.CtrOutz;
}

int main(int argc, char **argv)
{
setlocale(LC_ALL, "");

ros::init(argc, argv, "offb_node"); //ros初始化，最后一个参数为节点名称
ros::NodeHandle nh;


//订阅。<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为10）、回调函数
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);//订阅位置信息
//ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);//控制话题,用于发布位置信息
ros::Publisher vec_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);//速度控制话题,用于发布速度信息

ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
 
ros::Rate rate(20.0);
 
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
/*
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 0;
*/

	vector.linear.x = 0.0;
	vector.linear.y = 0.0;
	vector.linear.z = 0.0;

for(int i = 100; ros::ok() && i > 0; --i){
       vec_pub.publish(vector);
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
		//ROS_INFO("UAV is arming");//解锁后打印信息
            }
            last_request = ros::Time::now();
        }
	else
	{
		switch(step)
		{
			case 0:
				//pose.pose.position.x = 0;
				//pose.pose.position.y = 0;
				//pose.pose.position.z = 2;
				vec_pid(0,0,2,0);
				if (local_pos.pose.position.z > 1.9 && local_pos.pose.position.z < 2.1)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 1;
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
	 	                break;
			case 1:
				//pose.pose.position.x = 5;
				//pose.pose.position.y = 0;
				//pose.pose.position.z = 2;
				vec_pid(20,0,2,3);
				if (local_pos.pose.position.x > 19.9 && local_pos.pose.position.x < 20.1)
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
				//pose.pose.position.x = 5;
				//pose.pose.position.y = 5;
				//pose.pose.position.z = 2;
				vec_pid(20,20,2,3);
				if (local_pos.pose.position.y > 19.9 && local_pos.pose.position.y < 20.1)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 3;
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
	 	                break;
			case 3:
				//pose.pose.position.x = 0;
				//pose.pose.position.y = 5;
				//pose.pose.position.z = 2;
				vec_pid(0,20,2,3);
				if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 4;
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
	 	                break;
			case 4:
				//pose.pose.position.x = 0;
				//pose.pose.position.y = 0;
				//pose.pose.position.z = 2;
				vec_pid(0,0,2,3);
				if (local_pos.pose.position.x > -0.1 && local_pos.pose.position.x < 0.1 && local_pos.pose.position.y > -0.1 && local_pos.pose.position.y < 0.1)
                  		{
     		  	                if (sametimes > 20)
              				{
         		                   step = 5;
                        		}
                        		else
                           		sametimes++;
                    			}
                    			else sametimes = 0;
	 	                break;
			case 5:
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
    vec_pub.publish(vector);

    ros::spinOnce();
    rate.sleep();
}
 
return 0;

}
