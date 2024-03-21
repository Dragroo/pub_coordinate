#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
外参发布节点，监听从机械臂底座到摄像头
'''
from multiprocessing import dummy
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

class W2CPubDemo(Node):
    def __init__(self):
        super().__init__('bunker_w2c_pub')
        
        # self.declare_parameter('from_frame', 'Link6')
        self.declare_parameter('from_frame', 'base_link')
        self.from_frame = self.get_parameter('from_frame').get_parameter_value().string_value
        self.declare_parameter('to_frame', 'camera_link')
        # self.declare_parameter('to_frame', 'Link6')
        self.to_frame = self.get_parameter('to_frame').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.w2c_publisher = self.create_publisher(TransformStamped, 'w2c_transform', 50)
        
        self.timer = self.create_timer(0.1, self.on_timer)  # 1/50
        
    def on_timer(self):
        
        try:
            now = rclpy.time.Time() 
            time0 = Time()
            time0.sec = 0
            time0.nanosec = 0            
            #print(now)
            trans = self.tf_buffer.lookup_transform(
                self.to_frame,
                self.from_frame,
                now
            )
            # type: TransformStamped()
            # print(trans)
            # print(trans)
           
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.from_frame} to {self.to_frame}: {ex}'
            )
            return
        pos  = trans.transform.translation                          # 获取位置信息
        quat = trans.transform.rotation                             # 获取姿态信息（四元数）
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
          % (self.from_frame, self.to_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))
        self.w2c_publisher.publish(trans) 

def main(args=None):
    rclpy.init(args=args)
    cr3_w2c_pub = W2CPubDemo()
    rclpy.spin(cr3_w2c_pub)

if __name__=="__main__":
    main()
    