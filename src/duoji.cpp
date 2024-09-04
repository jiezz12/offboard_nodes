#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
	}
geometry_msgs::PoseStamped pose_uav;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	pose_uav = *msg;
	}
int main(int argc, char **argv)
{
ros::init(argc, argv, "wh_node");
ros::NodeHandle nh;
ros::Subscriber state_sub =nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
ros::Subscriber pose_sub =nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
ros::Publisher local_pos_pub =nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
ros::Publisher claw_pub_mix =nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
ros::ServiceClient arming_client =nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client =nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
ros::Rate rate(20.0);
while (ros::ok() &&!current_state.connected) {
	ros::spinOnce();
	rate.sleep();
	}
mavros_msgs::ActuatorControl claw_control_mix;
claw_control_mix.group_mix = 2;
//-1是最小转动角度 1是最大角度
for (int i = 0; i < 8; ++i) {
	claw_control_mix.controls[i] = -1;
	}
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0.0;
	pose.pose.position.y = 0.0;
	// pose.pose.position.z = 1.2;
	ros::Time last_request = ros::Time::now();
int state = 0;
while (ros::ok()) {
switch (state) {
case 0:
// Wait for offboard
	ROS_INFO("waiting for take off");
	if (current_state.mode !="OFFBOARD") {
		if (ros::Time::now() -last_request > ros::Duration(1.0)) {
			mavros_msgs::SetMode offb_set_mode;
			offb_set_mode.request.custom_mode ="OFFBOARD";
	if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
		ROS_INFO("Offboard enabled");
	}
		last_request =ros::Time::now();
    }
		local_pos_pub.publish(pose);
	} else {
		state = 1;
		}
break;
case 1:
// Wait for armed
	ROS_INFO("going to take off");
	if (!current_state.armed) {
		if (ros::Time::now() -last_request > ros::Duration(1.0)) {
			mavros_msgs::CommandBool arm_cmd;
			arm_cmd.request.value =true;
		if(arming_client.call(arm_cmd) &&arm_cmd.response.success) {
			ROS_INFO("Vehicle armed");
			claw_pub_mix.publish(claw_control_mix);
		}
			last_request =ros::Time::now();
	}
			local_pos_pub.publish(pose);
	} 
		else {
			state = 4;//初始化舵机角度
			claw_pub_mix.publish(claw_control_mix);
	}
break;
case 2:// Hover
	ROS_INFO("go to (0,0,0.1)");
	local_pos_pub.publish(pose);
	if(pose_uav.pose.position.z >0.1){
		state = 3;
	}
break;
case 3:
	pose.pose.position.x = 0.1;
	ROS_INFO("go to (1,0,1.2)");
	local_pos_pub.publish(pose);
	if(pose_uav.pose.position.x >0.05){
	state = 4;
	}
break;
case 4:
	//pose.pose.position.z = 0.02;
	ROS_INFO("go to (1,0,0.3)");
	local_pos_pub.publish(pose);
	//if(pose_uav.pose.position.z <0.02){
		for (int i = 0; i < 8; ++i)
		{//转动舵机 放下货物
			claw_control_mix.controls[i] = 1;
			claw_pub_mix.publish(claw_control_mix);
		}
	state = 5;
	//}
break;
case 5:
	//pose.pose.position.x = 0.0;pose.pose.position.z = 0.5;
	ROS_INFO("go to (0,0,0.5)");
	local_pos_pub.publish(pose);
	//if(pose_uav.pose.position.z >0.4 && pose_uav.pose.position.x < 0.1){
		for (int i = 0; i < 8; ++i)
		{//舵机回转
			claw_control_mix.controls[i] = -1;
			claw_pub_mix.publish(claw_control_mix);
		}
	state = 6;
	//}
break;
case 6:
// Land
	ROS_INFO("land");
	if (current_state.mode != "AUTO.LAND") {
		if (ros::Time::now() -last_request > ros::Duration(1.0)) {
			mavros_msgs::SetMode
			land_set_mode;
			land_set_mode.request.custom_mode ="AUTO.LAND";
	if(set_mode_client.call(land_set_mode) &&land_set_mode.response.mode_sent) {
		ROS_INFO("Vehicle landed");
	}
		last_request =ros::Time::now();
	}
	} 
	else {
		state = 7;
	     }
break;
case 7:
// End
	return 0;
		}
	ros::spinOnce();
	rate.sleep();
	}
	return 0;
}
