#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
负责通过机械臂的api获取从base_link到link6的变换，并以tf的形式发布，同时根据发送过来的偏移控制机械臂的移动
'''
from __future__ import print_function
import dobot_move.dobot_service_clients as dobot_service_clients
from dobot_move.dobot_service_clients import DobotPose
from my_interfaces.msg import Myuv
import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from std_msgs.msg import String                  # ROS2标准定义的String消息
import numpy as np
import math
from time import sleep
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scout_interfaces_msg.msg import ObjectWorld
import tf_transformations
"""
创建一个订阅者节点
"""
# 定时器设定的周期，单位s
TIMER_INTERVAL = 0.05
SWING_INTERVAL = 0.2
NEW_SWING_INTERVAL = 0.3


class SubscriberNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.object_subscriber = self.create_subscription(ObjectWorld, 'object_world_info', self.object_world_callback, 10)
        self.cr3_clients=dobot_service_clients.DobotClient("cr3_client")
        '''
        机械臂初始化部分
        '''
        
        self.dashboard, self.move, self.feed=self.cr3_clients.connect_robot()
        self.br = TransformBroadcaster(self)
        print("开始上电...")
        self.dashboard.PowerOn()
        print("请耐心等待,机器人正在努力启动中...")
        count = 10
        while count > 0 :
            print(count)
            count = count - 1
            sleep(1)
        print("开始使能...")
        # self.dashboard.EnableRobot()
        self.world_coordinate=ObjectWorld()
        self.swing_flag=False
        self.swing_index=0
        self.cam_offsetx=0.0
        self.swing_pose=[[41.835, 49.1, -74.2, 25.33, -31.7, 0.0],[41.835, 49.1, -74.2, 25.33, -51.7, 0.0]]
        self.dest_pose=[41.835, 49.1, -74.2, 25.33, -31.7, 0.0]
        self.tfpub_index=0
        self.current_pose=DobotPose()
        self.timer=self.create_timer(TIMER_INTERVAL,self.move_callback)
        self.swing_timer=self.create_timer(SWING_INTERVAL,self.swing_callback)
        # self.new_swing_timer=self.create_timer(NEW_SWING_INTERVAL,self.new_swing_callback)


    # def new_swing_callback(self):
    #     if(self.swing_flag==True):
    #         # s_index=self.swing_index
    #         # self.move.JointMovJ(self.swing_pose[s_index][0],self.swing_pose[s_index][1],self.swing_pose[s_index][2],
    #         #     self.swing_pose[s_index][3],self.swing_pose[s_index][4],self.swing_pose[s_index][5])
    #         # s_index^=1
    #         # self.swing_index=s_index
    #         self.move.MovJ(self.dest_pose[0],self.dest_pose[1],self.dest_pose[2],\
    #             self.dest_pose[3],self.dest_pose[4],self.dest_pose[5])

    def swing_callback(self):
        if(self.swing_flag==True):
            # self.move.RelMovJUser(self.cam_offsetx,0.0,0.0,0.0,0.0,0.0,0)
            self.move.JointMovJ(40.926, 49.509, -74.586, 25.494, -40.7+self.angle_offset, 0.0)
        # else:
        #     s_index=self.swing_index
        #     self.move.JointMovJ(self.swing_pose[s_index][0],self.swing_pose[s_index][1],self.swing_pose[s_index][2],
        #         self.swing_pose[s_index][3],self.swing_pose[s_index][4],self.swing_pose[s_index][5])
        #     s_index^=1
        #     self.swing_index=s_index
            # goal_pose=self.dashboard.PositiveSolution(40.926, 49.509, -74.586, 25.494, -40.7+self.angle_offset, 0.0)
            # goal_pose_val = []
            # goal_pose_val = self.cr3_clients.dat2li(goal_pose,'{','}')
            # print(goal_pose_val)
            # rx=math.radians(goal_pose_val[3])
            # ry=math.radians(goal_pose_val[4])
            # rz=math.radians(goal_pose_val[5])
            # pose1_matrix=tf_transformations.euler_matrix(rx,ry,rz,axes='sxyz')
            # pose1_matrix[0][3]=goal_pose_val[0]*1e-3
            # pose1_matrix[1][3]=goal_pose_val[1]*1e-3
            # pose1_matrix[2][3]=goal_pose_val[2]*1e-3
            # pose2_matrix=tf_transformations.euler_matrix(0.0,0.0,0.0,axes='sxyz')
            # if(self.cam_offsetx>8.0):
            #     pose2_matrix[2][3]=(self.cam_offsetx-8.0)*0.05
            # else:
            #     pose2_matrix[2][3]=0.0
            # print(pose2_matrix)
            # dest_matrix=np.matmul(pose1_matrix,pose2_matrix)
            # dest_angle=tf_transformations.euler_from_matrix(dest_matrix,axes='sxyz')
            # self.dest_pose=[(dest_matrix[0][3])*1e3,dest_matrix[1][3]*1e3,dest_matrix[2][3]*1e3,\
            #             math.degrees(dest_angle[0]),math.degrees(dest_angle[1]),math.degrees(dest_angle[2])]
            # print(dest_pose)
            


    def object_world_callback(self,object_world_msg):
        self.swing_flag=object_world_msg.move_flag
        angle_offset=math.degrees(object_world_msg.cam2person_angle)
        self.angle_offset=angle_offset
        self.cam_offsetx=object_world_msg.offset_x

    def move_callback(self):
        '''
        用于发布机械臂当前位姿,转换成tf输出
        '''
        # if(self.tfpub_index==0):
        #     self.current_pose=self.cr3_clients.get_feed(self.feed)
        # self.current_pose=current_pose
        current_pose=self.dashboard.GetPose()
        pos_val = []
        pos_val = self.cr3_clients.dat2li(current_pose,'{','}')
        # print(pos_val)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        '''
        这里的位姿变换有一点问题，实际上不能直接加，因为这样是相对世界坐标系的偏移，实际上应该是相对相机坐标系的偏移
        '''
        t.transform.translation.x = pos_val[0]*1e-3
        t.transform.translation.y = pos_val[1]*1e-3-0.07
        t.transform.translation.z = pos_val[2]*1e-3+0.06
        quat = tf_transformations.quaternion_from_euler(math.radians(pos_val[3]),math.radians(pos_val[4]),math.radians(pos_val[5]),axes='sxyz')
        # t.transform.translation.x = self.current_pose.x*1e-3
        # t.transform.translation.y = self.current_pose.y*1e-3-0.07
        # t.transform.translation.z = self.current_pose.z*1e-3+0.06
        # print(self.current_pose.x,self.current_pose.y,self.current_pose.z)
        # quat = tf_transformations.quaternion_from_euler(math.radians(self.current_pose.rx),math.radians(self.current_pose.ry),math.radians(self.current_pose.rz),axes='sxyz')
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)
        # if(self.swing_flag):
        #     if(self.swing_index==0):
        #         print(40.926, 49.509, -74.586, 25.494, -40.7+self.angle_offset, 0.0)
        #         self.move.JointMovJ(40.926, 49.509, -74.586, 25.494, -40.7+self.angle_offset, 0.0)
        #     if(self.swing_index==39):
        #         self.swing_index=0
        #     else:
        #         self.swing_index += 1
        #     print("swing_flag true!")
        # if(self.tfpub_index==9):
        #     self.tfpub_index = 0
        # else:
        #     self.tfpub_index += 1

        # print(t)

        
    # def tf_callback(self):

    #     '''
    #     用于发布机械臂当前位姿，转换成tf输出
    #     '''
    #     current_pose=self.dashboard.GetPose()
    #     pos_val = []
    #     pos_val = self.cr3_clients.dat2li(current_pose,'{','}')
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'base_link'
    #     t.child_frame_id = 'Link6'
    #     t.transform.translation.x = pos_val[0]*10e-3
    #     t.transform.translation.y = pos_val[1]*10e-3
    #     t.transform.translation.z = pos_val[2]*10e-3
    #     quat = tf_transformations.quaternion_from_euler(math.radians(pos_val[3]),math.radians(pos_val[4]),math.radians(pos_val[5]),axes='sxyz')
    #     t.transform.rotation.x = quat[0]
    #     t.transform.rotation.y = quat[1]
    #     t.transform.rotation.z = quat[2]
    #     t.transform.rotation.w = quat[3]
    #     self.br.sendTransform(t)
        
def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                    # ROS2 Python接口初始化
    node = SubscriberNode("pose_suber_node")    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口