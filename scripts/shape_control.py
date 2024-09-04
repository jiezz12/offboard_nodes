#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import mavros_msgs
import rospy
from mavros_msgs.srv import SetMode,CommandBool

import sys, select, os
#os模块是用于与操作系统进行交互的模块。TTY（teletypewriter）是指终端设备，例如终端窗口、终端控制台或串口终端。
#select 模块：该模块提供了对低级 I/O 多路复用的支持。
#termios模块是Python中用于处理终端IO（Input/Output）的模块，允许我们控制终端的特性（attributes），例如字符的读取方式、输入输出模式等。
import tty, termios
from std_msgs.msg import String

msg2all = """
请输入输入指令:
1~9控制编队形状
r   : return home
t/y : arm/disarm
v/n : takeoff/land
b   : offboard
p   : position
s   : stabilized
k   : hover and remove the mask of keyboard control
CTRL-C to quit
"""





def getKey():
    #这行代码使用tty模块的setraw()函数来设置标准输入（sys.stdin）的行为为原始模式。
    #原始模式下，输入不经过缓冲，每次输入一个字符。
    tty.setraw(sys.stdin.fileno())
    #这行代码使用select模块的select()函数来检查是否有可读取的数据。
    #它监视sys.stdin（标准输入），并且等待0.1秒钟。如果在等待期间有数据可读，
    #则select()函数会返回一个非空的可读列表（rlist），否则返回空列表
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    #这段代码检查rlist列表是否非空。如果列表非空，说明在等待期间有数据可读。
    #此时，通过sys.stdin.read(1)读取一个字符，并将其赋值给key变量。
    #如果列表为空，则说明在等待期间没有数据可读，此时将key变量赋值为空字符串。
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    #这行代码使用termios模块的tcsetattr()函数来恢复标准输入的设置。
    #TCSADRAIN参数表示在所有排队的输出都被传输和处理之后才生效。
    #将标准输入的属性设置为之前保存的settings值。它使用termios.tcsetattr函数来设置终端的属性。
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    #最后，函数返回变量key的值，即读取到的字符（如果有）或空字符串（如果没有读取到字符）。
    return key


def print_msg():
        print(msg2all)


#主函数
if __name__=="__main__":
#设置终端为标准输入流
    settings = termios.tcgetattr(sys.stdin)
    uav_name = "uav" + std::to_string(i);
    print_msg()
    cmd=""
    rospy.init_node('my_keyboard_control')

	uav_cmd_pub = rospy.ServiceProxy(uav_name+"/mavros/set_mode",SetMode)
        uav_arm_pub=rospy.ServiceProxy(uav_name+"/mavros/cmd/arming",mavros_msgs.srv.CommandBool)


    while(1):
        

        key = getKey()#获取读取到的字符
        
	if key == '1':
            uav_name = 'uav0'
	    print("开启无人机一号")
        elif key == '2':
            uav_name = 'uav1'
	    print("开启无人机二号")
        elif key == '3':
            uav_name = 'uav2'
    	    print("开启无人机三号")
	elif key == '4':
            uav_name = 'uav3'
    	    print("开启无人机四号")
	elif key == '5':
            uav_name = 'uav4'
    	    print("开启无人机五号")
	elif key == '6':
            uav_name = 'uav5'
    	    print("开启无人机六号")
	elif key == '7':
            uav_name = 'uav6'
    	    print("开启无人机七号")
	elif key == '8':
            uav_name = 'uav7'
    	    print("开启无人机八号")
	elif key == '9':
            uav_name = 'uav8'
    	    print("开启无人机九号")



        if key == 'r':
            cmd = 'AUTO.RTL'
            print_msg()
            print('Returning home')
        elif key == 't':
            cmd = 'ARM'
            print_msg()
            print('Arming')
        elif key == 'y':
            cmd = 'DISARM'
            print_msg()
            print('Disarming')
        elif key == 'v':
            cmd = 'AUTO.TAKEOFF'
            print_msg()
            #print('Takeoff mode is disenabled now')
        elif key == 'b':
            cmd = 'OFFBOARD'
            print_msg()
            print('Offboard')
        elif key == 'n':
            cmd = 'AUTO.LAND'
            print_msg()
            print('Landing')
        elif key == 'p':
            cmd = 'POSCTL'
            print_msg()
            print('Position')
        elif key == 's':
            cmd = 'STABILIZED'
            print_msg()
            print('Stabilized')
        elif key in ['k']:
            cmd = 'HOVER'
            print_msg()
            print('Hover')
        elif(key == '\x03'):
                break

        if (cmd=='ARM'):
            uav_arm_pub(True)
        elif (cmd=='DISARM'):
            uav_arm_pub(False)
        else:
            uav_cmd_pub(custom_mode=cmd)


        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

