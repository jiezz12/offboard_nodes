/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <vector>

struct SwarmParameters {
    int swarm_size_n = 4;
    const double k1 = 0.5;
    const double k2 = 0.5;
    double A_i_bar[4][4] = {{0.0, 1.0, 1.0, 1.0}, {1.0, 0.0, 1.0, 1.0}, {1.0, 1.0, 0.0, 1.0}, {1.0, 1.0, 1.0, 0.0}};
    double H_i_bar[4][3] = {{1.0, 0.0, 0.5}, {1.0, 1.0, 0.0}, {0.0, 1.0, -0.5}, {0.0, 0.0, 0.0}};
    std::vector<geometry_msgs::PoseStamped> current_poses;
    std::vector<geometry_msgs::TwistStamped> current_velocities;
    std::vector<mavros_msgs::State> current_states;

    SwarmParameters() : current_poses(swarm_size_n), current_velocities(swarm_size_n), current_states(swarm_size_n) {}
};

SwarmParameters swarm_parameters;

void state_cb(const mavros_msgs::State::ConstPtr& msg, int uav_index) {
    swarm_parameters.current_states[uav_index] = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int uav_index) {
    swarm_parameters.current_poses[uav_index] = *msg;
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg, int uav_index) {
    swarm_parameters.current_velocities[uav_index] = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    std::vector<ros::Subscriber> state_subs(4), local_pos_subs(4), local_vel_subs(4);
    std::vector<ros::Publisher> setpoint_pubs(4);
    std::vector<ros::ServiceClient> arming_clients(4), mode_clients(4);


    // 初始化订阅器、发布器和服务客户端
    for (int i = 0; i < 4; ++i) {
        std::string uav_name = "uav" + std::to_string(i);
        state_subs[i] = nh.subscribe<mavros_msgs::State>(uav_name + "/mavros/state", 10, boost::bind(state_cb, _1, i));
        local_pos_subs[i] = nh.subscribe<geometry_msgs::PoseStamped>(uav_name + "/mavros/local_position/pose", 10, boost::bind(local_pos_cb, _1, i));
        local_vel_subs[i] = nh.subscribe<geometry_msgs::TwistStamped>(uav_name + "/mavros/local_position/velocity_local", 10, boost::bind(local_vel_cb, _1, i));
        setpoint_pubs[i] = nh.advertise<mavros_msgs::PositionTarget>(uav_name + "/mavros/setpoint_raw/local", 10);
        arming_clients[i] = nh.serviceClient<mavros_msgs::CommandBool>(uav_name + "/mavros/cmd/arming");
        mode_clients[i] = nh.serviceClient<mavros_msgs::SetMode>(uav_name + "/mavros/set_mode");
    }

    ros::Rate rate(20.0);
    //定义起飞高度
    const float takeoff_height = 2.0;
    bool is_offboard_started = false;
    bool takeoff_completed = false;
    ros::Time last_mode_change = ros::Time::now();


    // 等待飞控连接
    bool all_connected = false;
    while (ros::ok() && !all_connected) {
        all_connected = true;
        for (int i = 0; i < 4; ++i) {
            if (!swarm_parameters.current_states[i].connected) {
                all_connected = false;
                 ROS_INFO("UAV %d is not connected.", i);
                break;
            }else{
                ROS_INFO("UAV %d is connected.", i);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    // 解锁和设置模式
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

	mavros_msgs::PositionTarget fake_pose[4];
    // 发送一些虚拟的设定点，以便无人机切换到OFFBOARD模式
    for (int i = 0; i < 4; ++i) {
    //geometry_msgs::PoseStamped fake_pose;
    //fake_pose.pose.position.x = i % 2;  // 0 或 1
    //fake_pose.pose.position.y = i / 2;  // 0 或 1
    //fake_pose.pose.position.z = 2.0;

	fake_pose[i].header.stamp = ros::Time::now();	//存储ROS中的时间戳信息
	fake_pose[i].coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	fake_pose[i].type_mask =			//使用位置控制
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
	fake_pose[i].position.x = 0 ;
	fake_pose[i].position.y = 0 ;
	fake_pose[i].position.z = 0 ;	
	
    for (int j = 0; j < 10; ++j) {
        setpoint_pubs[i].publish(fake_pose[i]);  // 使用发布者发布虚拟位置
	//ros::spinOnce();
        rate.sleep();
    }
/*
if (!swarm_parameters.current_states[i].armed) {
        arming_clients[i].call(arm_cmd);
    }
	if (swarm_parameters.current_states[i].mode != "OFFBOARD") {
        mode_clients[i].call(offb_set_mode);
    }
*/
}
	

for(int i = 0;i < 4; ){
	
    if (swarm_parameters.current_states[i].mode != "OFFBOARD" ) {
        mode_clients[i].call(offb_set_mode);
	//last_mode_change = ros::Time::now();
    }
	if (!swarm_parameters.current_states[i].armed) {
        arming_clients[i].call(arm_cmd);
	//last_mode_change = ros::Time::now();
    }
	if(arm_cmd.response.success && offb_set_mode.response.mode_sent) i++;
}
	
for (int i = 0; i < 4; ++i) {
	fake_pose[i].position.x = 0 ;
	fake_pose[i].position.y = 0 ;
	fake_pose[i].position.z = 2;
    for (int j = 0; j < 10; ++j) {
        setpoint_pubs[i].publish(fake_pose[i]);  // 使用发布者发布虚拟位置
	ros::spinOnce();
        rate.sleep();
    }
}
    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;
    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    // for (int i = 0; i < 4; ++i) {
    //     if (!swarm_parameters.current_states[i].armed) {
    //         arming_clients[i].call(arm_cmd);
    //     }
    //     if (swarm_parameters.current_states[i].mode != "OFFBOARD") {
    //         mode_clients[i].call(offb_set_mode);
    //     }
    // }


    //uav0一直向前的相关参数
     float speed = 1.0; // 设定无人机的速度
     ros::Time start_time = ros::Time::now();
    //  bool is_offboard_started = false;
     ROS_INFO("Initializing UAV control node...");

     while (ros::ok()) {
        // 检查所有无人机是否都在 OFFBOARD 模式且已解锁
        bool all_offboard_and_armed = true;
        for (int i = 0; i < 4; ++i) {
            if (!(swarm_parameters.current_states[i].mode == "OFFBOARD" && swarm_parameters.current_states[i].armed)) {
                all_offboard_and_armed = false;
                break;
            }
        }

        if (all_offboard_and_armed) {
            if (!is_offboard_started) {
                is_offboard_started = true;
                last_mode_change = ros::Time::now();
                ROS_INFO("All UAVs are now in OFFBOARD mode and armed.");
            }

            if (!takeoff_completed && (ros::Time::now() - last_mode_change).toSec() > 10.0) {
		//if(!takeoff_completed &&)
                // 假设无人机已经到达预定高度
                takeoff_completed = true;
                ROS_INFO("Takeoff completed for all UAVs.");
            } else if (!takeoff_completed) {
                // 无人机起飞到预定高度的逻辑
                for (int i = 0; i < 4; ++i) {
                    //mavros_msgs::PositionTarget pose;
                    //pose.position.z = takeoff_height;
			fake_pose[i].position.x = 0 ;
			fake_pose[i].position.y = 0 ;
			fake_pose[i].position.z = 2 ;
                    setpoint_pubs[i].publish(fake_pose[i]);
                }
            }


            // uav0 向前飞行的逻辑
            if (takeoff_completed && (ros::Time::now() - last_mode_change).toSec() > 15.0) {
            ROS_INFO("Executing forward motion for uav0.");
            mavros_msgs::PositionTarget setpoint0;
            setpoint0.header.stamp = ros::Time::now();
            setpoint0.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // 仅使用速度控制
            setpoint0.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                  mavros_msgs::PositionTarget::IGNORE_PY |
                                  mavros_msgs::PositionTarget::IGNORE_PZ |
                                //   mavros_msgs::PositionTarget::IGNORE_VX |
                                //   mavros_msgs::PositionTarget::IGNORE_VY |
                                //   mavros_msgs::PositionTarget::IGNORE_VZ |
                                  mavros_msgs::PositionTarget::IGNORE_AFX |
                                  mavros_msgs::PositionTarget::IGNORE_AFY |
                                  mavros_msgs::PositionTarget::IGNORE_AFZ |
                                  mavros_msgs::PositionTarget::FORCE |
                                  mavros_msgs::PositionTarget::IGNORE_YAW |
                                  mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            setpoint0.velocity.x = speed;
            setpoint0.velocity.y = 0;
            setpoint0.velocity.z = 0;
            setpoint_pubs[0].publish(setpoint0);
        }else {
                // 无人机起飞到预定高度的逻辑
                for (int i = 0; i < 4; ++i) {
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.z = takeoff_height;
                    setpoint_pubs[i].publish(pose);
                }
            }
        } 
        // 一致性算法逻辑
        for (int i = 1; i < 4; ++i) {
            mavros_msgs::PositionTarget setpoint;
            setpoint.header.stamp = ros::Time::now();
            setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                 mavros_msgs::PositionTarget::IGNORE_PY |
                                 mavros_msgs::PositionTarget::IGNORE_PZ |
                                 mavros_msgs::PositionTarget::IGNORE_AFX |
                                 mavros_msgs::PositionTarget::IGNORE_AFY |
                                 mavros_msgs::PositionTarget::IGNORE_AFZ |
                                 mavros_msgs::PositionTarget::FORCE |
                                 mavros_msgs::PositionTarget::IGNORE_YAW |
                                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            setpoint.velocity.x = 0;
            setpoint.velocity.y = 0;
            setpoint.velocity.z = 0;

            for (int j = 0; j < 4; ++j) {
                if (i == j) continue; // 跳过与自己的比较

                double dx = swarm_parameters.current_poses[j].pose.position.x - swarm_parameters.current_poses[i].pose.position.x;
                double dy = swarm_parameters.current_poses[j].pose.position.y - swarm_parameters.current_poses[i].pose.position.y;
                double dvx = swarm_parameters.current_velocities[j].twist.linear.x - swarm_parameters.current_velocities[i].twist.linear.x;
                double dvy = swarm_parameters.current_velocities[j].twist.linear.y - swarm_parameters.current_velocities[i].twist.linear.y;

                setpoint.velocity.x += (swarm_parameters.k1 * (dx + swarm_parameters.H_i_bar[i][0]) + swarm_parameters.k2 * dvx) * swarm_parameters.A_i_bar[i][j];
                setpoint.velocity.y += (swarm_parameters.k1 * (dy + swarm_parameters.H_i_bar[i][1]) + swarm_parameters.k2 * dvy) * swarm_parameters.A_i_bar[i][j];
            }

            setpoint_pubs[i].publish(setpoint);
            // ROS_INFO("Publishing setpoint for uav%d.", i);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
