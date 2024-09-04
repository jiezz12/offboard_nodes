#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from mavros_msgs.msg import ActuatorControl

def control_px4():
    # 初始化ROS节点
    rospy.init_node('px4_controller', anonymous=True)

    # 创建一个发布器，发布到ActuatorControl话题
    actuator_control_pub = rospy.Publisher('/uav_0/mavros/actuator_control', ActuatorControl, queue_size=10)

    # 设置发布的消息
    actuator_control_msg = ActuatorControl()

    # 设置消息的控制模式和数据
    actuator_control_msg.group_mix = 0  # 0表示使用设置的控制值
    actuator_control_msg.controls = [0.0] * 8  # 8个控制通道，根据需要调整

    # 设置发布频率
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # 更新控制值
        # 在这里添加你的控制逻辑，根据需要修改actuator_control_msg.controls的值

        # 发布消息
        actuator_control_pub.publish(actuator_control_msg)

        # 等待一段时间，以满足发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        control_px4()
    except rospy.ROSInterruptException:
        pass

