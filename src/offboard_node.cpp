#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/SetMode.h>     
#include <mavros_msgs/State.h>

//建立一个订阅消息体类型的变量
mavros_msgs::State current_state;

//订阅时的回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 

int main(int argc, char **argv)
{
ros::init(argc, argv, "offb_node"); //ros初始化，最后一个参数为节点名称
ros::NodeHandle nh;

//订阅。<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为10）、回调函数
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
 
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);//控制话题,用于发布位置信息
 
//启动服务1，设置客户端（Client）名称为arming_client，客户端的类型为ros::ServiceClient，用于请求解锁无人机
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

//启动服务2，设置客户端（Client）名称为set_mode_client，客户端的类型为ros::ServiceClient，用于请求切换无人机模式
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
 
//发布频率
ros::Rate rate(20.0);
 
// 等待飞控连接mavros，current_state是我们订阅的mavros的状态，连接成功在跳出循环
while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
}
//先实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
geometry_msgs::PoseStamped pose;
pose.pose.position.x = 0;
pose.pose.position.y = 0;
pose.pose.position.z = 2;
 
//启动之前发送一些设定值
for(int i = 100; ros::ok() && i > 0; --i){
       local_pos_pub.publish(pose);
       ros::spinOnce();
       rate.sleep();
}
//建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的客户端与服务端之间的通信（服务）
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
 
//建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的客户端与服务端之间的通信（服务）
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;
 
//更新时间
ros::Time last_request = ros::Time::now();
 
while(ros::ok())//进入大循环
{
    //首先判断当前模式是否为offboard模式，如果不是，则客户端set_mode_client向服务端offb_set_mode发起请求call
    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
       	if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");//打开模式后打印信息
        }
        last_request = ros::Time::now();
    }
    else //else指已经为offboard模式，然后进去判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call
    {
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("UAV is arming");//解锁后打印信息
            }
            last_request = ros::Time::now();
        }
    }
 
    local_pos_pub.publish(pose); //发布位置信息，所以综上飞机只有先打开offboard模式然后解锁才能飞起来
 
    ros::spinOnce();
    rate.sleep();
}
 
return 0;

}
