#!/usr/bin/env python
#coding=utf-8

import rospy
from sensor_msgs.msg import Imu,Range  #姿态角 ，雷达
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped#位置坐标
from darknet_ros_msgs.msg import BoundingBoxes
import yaml
import os
import numpy as np
import math

#读取参数
curPath = os.path.dirname(os.path.realpath(__file__))
yamlPath = os.path.join(curPath, "../config/config2.yaml")
f = open(yamlPath, 'r')
cfg = f.read()
d = yaml.load(cfg)


camera_positon = d['camera_positon']
camera_positon =np.array([[camera_positon[0]],[camera_positon[1]],[camera_positon[2]]])

mtx = np.array([[d['fx'],0.0,d['ux']],[0.0,d['fy'],d['uy']],[0.0,0.0,1.0]])#相机内参
dist = np.array([[d['k1']],[d['k2']],[d['p1']],[d['p2']],[d['k3']]])    #畸变参数

def imu_callback(msg):
    if msg.orientation_covariance[0] < 0:
        return
    quaternion = [
        msg.orientation.x , 
        msg.orientation.y ,
        msg.orientation.z ,
        msg.orientation.w
    ]
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    #roll =roll*180/math.pi
    #pitch = pitch*180/math.pi
    #yaw = yaw*180/math.pi
    # rigbt(roll,pitch,yaw)
   # rospy.loginfo("滚转角=%f,俯仰角=%f,朝向=%f",roll,pitch,yaw)

def darknet_callback(msg):
    global pix_x, pix_y
    for bbox in msg.bounding_boxes:
        x_min = bbox.xmin
        y_min =  bbox.ymin
        x_max = bbox.xmax
        y_max = bbox.ymax

    pix_x = (x_max + x_min)/2
    pix_y = (y_max + y_min)/2
    
    # rospy.loginfo("xmin=%f,xmax=%f,ymin=%f,ymax=%f",x_min,x_max,y_min,y_max)

def px4_callback(px4_msg):   
    global x0,y0,z0
    zuobiao_px4 = px4_msg.pose.position
    x0 = zuobiao_px4.x
    y0 = zuobiao_px4.y
    z0 = zuobiao_px4.z

def rigbt(Phi, Psi, Theta):     #旋转矩阵
    # global R, t, tvec
    R1 = np.array([[1, 0, 0],
                   [0, math.cos(Phi), math.sin(Phi)],
                   [0, -math.sin(Phi), math.cos(Phi)]])
    R2 = np.array([[math.cos(Psi), 0, -math.sin(Psi)],
                   [0, 1, 0],
                   [math.sin(Psi), 0, math.cos(Psi)]])
    R3 = np.array([[math.cos(Theta), -math.sin(Theta), 0],
                   [math.sin(Theta), math.cos(Theta), 0],
                   [0, 0, 1]])
    R = np.dot(np.dot(R1, R2), R3)    
    #print(R)
    return R
def uv2camera (u,v,camera_matrix,distortion_cofficients,depth):

    fx_ = camera_matrix[0, 0]
    fy_ = camera_matrix[1, 1]
    cx_ = camera_matrix[0, 2]
    cy_ = camera_matrix[1, 2]

    k1 = distortion_cofficients[0, 0]
    k2 = distortion_cofficients[1, 0]
    p1 = distortion_cofficients[2, 0]
    p2 = distortion_cofficients[3, 0]
    k3 = distortion_cofficients[4, 0]

    # k1 = 0
    # k2 = 0
    # p1 = 0
    # p2 = 0
    # k3 = 0

    x = (u - cx_) * 1.0 / fx_
    y = (v - cy_) * 1.0 / fy_
    z = 1.0
    r_2 = x * x + y * y
    x_distorted = x * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + 2 * p1 * x * y + p2 * (r_2 + 2 * x * x)
    y_distorted = y * (1 + k1 * r_2 + k2 * r_2 * r_2 + k3 * r_2 * r_2 * r_2) + p1 * (r_2 + 2 * y * y) + 2 * p2 * x * y

    p_c = np.array([[x_distorted*depth],[y_distorted*depth],[z*depth]])
    #print(" p_c", p_c)

    return p_c


def camera2world (p_c,R,tvec):

    T = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])  #外参矩阵
    T[0, 0] = R[0, 0]
    T[0, 1] = R[0, 1]
    T[0, 2] = R[0, 2]
    T[1, 0] = R[1, 0]
    T[1, 1] = R[1, 1]
    T[1, 2] = R[1, 2]
    T[2, 0] = R[2, 0]
    T[2, 1] = R[2, 1]
    T[2, 2] = R[2, 2]
    T[0, 3] = tvec[0, 0]
    T[1, 3] = tvec[1, 0]
    T[2, 3] = tvec[2, 0]
    T[3, 0] = 0.0
    T[3, 1] = 0.0
    T[3, 2] = 0.0
    T[3, 3] = 1.0
    tmp = np.array([[0.0], [0.0], [0.0], [1.0]])
    tmp[0, 0] = p_c[0, 0]
    tmp[1, 0] = p_c[1, 0]
    tmp[2, 0] = p_c[2, 0]
    T_inv = np.linalg.inv(T)  #求逆矩阵
    p_w = np.dot(T_inv,tmp) #求矩阵相乘
    p_w_3D = np.array([[0.0], [0.0], [0.0]])#初始化矩阵
    p_w_3D[0, 0] = p_w[0, 0]
    p_w_3D[1, 0] = p_w[1, 0]
    p_w_3D[2, 0] = p_w[2, 0]

    return p_w_3D

def uv2world(u, v, camera_matrix, distortion_cofficients, depth ,R,tvec):

    p_c = uv2camera(u, v, camera_matrix, distortion_cofficients, depth)
    p_w_3D = camera2world(p_c, R, tvec)

    return p_w_3D

def Cal_n3_2(u,v,z):
    R = rigbt(roll,pitch,yaw)
    t = - np.matrix(R) * np.matrix(camera_positon)  #平移矩阵
    tvec = np.array([[t[0]],[t[1]],[t[2]]])
    point_test = uv2world(u,v, mtx, dist, 1, R, tvec)
    point_test_2 = uv2world(u,v, mtx, dist, 2, R, tvec)

    tmp_x = point_test_2[0, 0] - point_test[0, 0]
    tmp_y = point_test_2[1, 0] - point_test[1, 0]
    tmp_z = point_test_2[2, 0] - point_test[2, 0]
    n3 = (z - (point_test[2, 0])) / tmp_z
    p_w_3D = np.array([[0.0], [0.0], [0.0]])
    p_w_3D[0, 0] = point_test[0, 0] + n3 * tmp_x
    p_w_3D[1, 0] = point_test[1, 0] + n3 * tmp_y
    p_w_3D[2, 0] = z

    return p_w_3D

def getPos(U, V):
    global point_world_D
    Z = 0.0; #根据坐标系提前确定高程值 
    point_world_D = Cal_n3_2(U,V,Z)
    rospy.loginfo("x=%f,y=%f", x0+point_world_D[0,0],y0+point_world_D[1,0])

    return point_world_D

if __name__ == "__main__":
    rospy.init_node("pix2world")

    imu_sub = rospy.Subscriber('/mavros/imu/data', Imu,imu_callback,queue_size=10)
    darknet_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,darknet_callback, queue_size=10)
    px4_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,px4_callback,queue_size=10 )
    
    pix_x = 0
    pix_y = 0
    roll = 0
    pitch = 0
    yaw = 60*math.pi/180
    x0,y0,z0 = 0,0,0
    while not rospy.is_shutdown():    
        getPos(pix_x,pix_y)
    rospy.spin()