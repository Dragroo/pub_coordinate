import threading
from dobot_move.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np
import re

class DobotPose:
    def __init__(self):
        '''
        初始位姿
        '''
        DobotPose.x=0.0
        DobotPose.y=-233.3
        DobotPose.z=754.8
        DobotPose.rx=-90.0
        DobotPose.ry=0.0
        DobotPose.rz=-180.0

class DobotClient:
    def __init__(self,name):
        self.name=name
    def connect_robot(self):
        try:
            #ip = "192.168.5.1" 
            ip = "192.168.1.6"
            dashboard_p = 29999
            move_p = 30003
            feed_p = 30004
            print("正在建立连接...")
            dashboard = DobotApiDashboard(ip, dashboard_p)
            move = DobotApiMove(ip, move_p)
            feed = DobotApi(ip, feed_p)
            print(">.<连接成功>!<")
            return dashboard, move, feed
        except Exception as e:
            print(":(连接失败:(")
            raise e

    def jointMovJ(self, move: DobotApiMove, point_list: list):
        '''
        点动(关节)
        '''
        move.JointMovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5])

    def movJ(self, move: DobotApiMove, point_list: list):
        '''
        点动(坐标)
        '''
        move.MovJ(point_list[0], point_list[1], point_list[2], point_list[3], point_list[4], point_list[5])
    
    def get_feed(self, feed: DobotApi):
        '''
        获取当前位姿，也可以使用GetPose()
        '''
        current_pose=DobotPose()
        hasRead = 0
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0

        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':

            # Refresh Properties
            current_actual = a["tool_vector_actual"][0]
            # print("tool_vector_actual:", current_actual)
            current_pose.x=current_actual[0]
            current_pose.y=current_actual[1]
            current_pose.z=current_actual[2]
            current_pose.rx=current_actual[3]
            current_pose.ry=current_actual[4]
            current_pose.rz=current_actual[5]
            
        return current_pose
    def dat2li(self,str,begin_str,end_str):
        '''
        数据转列表
        '''
        # restr="["+begin_str+"].*["+end_str+"]"
        # p1 = re.compile(restr)
        # p2 = (re.findall(pattern=p1,string=str))[0]
        # substr=begin_str+'|'+end_str
        # p2 = re.sub(substr,'',p2)
        ostring=str.split(end_str,1)[0]
        istring=ostring.split(begin_str,1)[1]
        li = istring.split(',')
        for index in range(len(li)):
            li[index]=float(li[index])     
        return li
        
    