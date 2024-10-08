#!/usr/bin/env python

import cv2
import subprocess
import socket
 
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('8.8.8.8', 80))
ip = s.getsockname()[0]
print("ip:",ip)
#上述是动态获取本地ip
rtsp = "rtsp://"+ip+":8554/back"
rtmp='rtmp://localhost:1935/mylive/test'
 #rtmp是推流的远端地址
# 读取视频并获取属性
cap = cv2.VideoCapture(rtsp)
size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
sizeStr = str(size[0]) + 'x' + str(size[1])
 
command = ['ffmpeg',
    '-y', '-an',
    '-f', 'rawvideo',
    '-vcodec','rawvideo',
    '-pix_fmt', 'bgr24',
    '-s', sizeStr,
    '-r', '25',
    '-i', '-',
    '-c:v', 'libx264',
    '-pix_fmt', 'yuv420p',
    '-preset', 'ultrafast',
    '-f', 'flv',
    rtmp]
 
pipe = subprocess.Popen(command
    , shell=False
    , stdin=subprocess.PIPE
)
 
while cap.isOpened():
    success,frame = cap.read()
    if success:
        '''
		对frame进行识别处理
		'''
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break    
        pipe.stdin.write(frame.tostring())
 
cap.release()
pipe.terminate()

