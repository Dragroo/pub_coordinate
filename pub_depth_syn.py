import numpy as np
import cv2
import rclpy
from rclpy.node import Node
import tf_transformations
from scout_interfaces_msg.msg import ObjectWorld
from scout_interfaces_msg.msg import ObjectPointsMixed
from scout_interfaces_msg.msg import ObjectPoints2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from scipy import linalg
import pyrealsense2 as rs
import math
# camera matrix
# 588.438619 0.000000 334.597566
# 0.000000 784.017764 208.232343
# 0.000000 0.000000 1.000000

# distortion
# -0.204674 0.160266 -0.000775 0.000662 0.000000


# H=Zh-Zl=r(-1)[31]*(xch-xcl)+r(-1)[32]*(Ych-ycl)
FACT_Z=0.912
PERSON_HEIGHT=1.70
# inmatrix = np.array([[588.438619,0.000000,334.597566],[0.000000,784.017764,208.232343],[0.000000,0.000000,1.000000]])
inmatrix = np.array([[588.438619,0.000000,300.597566],[0.000000,784.017764,208.232343],[0.000000,0.000000,1.000000]])
inmatrix1 = np.array([[576.148294,0.0,300.368802,0.0],[0.0,768.959677,258.827383,0.0],[0.0,0.0,1.0,0.0]])
distort_param = np.array([-0.204674,0.160266,-0.000775,0.000662,0.000000])
class WorldVectorPub(Node):

    def __init__(self):
        super().__init__('WorldVectorPub')
        self.tf_w2c_subscriber = self.create_subscription(ObjectPointsMixed, 'pixel_points_mixed',self.object_world_compute,10)
        self.to_pub=ObjectWorld()
        self.object_world_pub = self.create_publisher(ObjectWorld, 'object_world_info', 10)

    def object_world_compute(self, msg):
        # self.get_logger().info('I heard a message')
        person_height=1.70

        if(abs((msg.u1+msg.u2)/2-320)/640>0.45):
            print("像素点位于边缘")
            self.to_pub.move_flag=False
            self.object_world_pub.publish(self.to_pub)
        elif(abs(msg.u1-msg.u2)>100):
            print("目标过近")
            self.to_pub.move_flag=False
            self.object_world_pub.publish(self.to_pub)
        else:
            '''
            处理外参，转换成矩阵
            '''
            # if(msg.object_label=="head"):
            #     person_height=0.40
            # elif(msg.object_lable=="body"):
            #     person_height=1.70
            # 计算world_to_camera 齐次变换矩阵  
            # tf_extend.world_to_camera_pub中获取的四元数参考坐标系是"camera"，因此转化的矩阵就是从world到camera的
            w2c_1 = msg.pixel_w2c
            # print(w2c_1)
            # print("w2c_1: {}".format(w2c_1))
            # w2c_2 = pixel_msg.pixel2_w2c
            # print("w2c_2: {}".format(w2c_1))

            
            w2c_1_li = []
            w2c_1_li.append(w2c_1.rotation.x)
            w2c_1_li.append(w2c_1.rotation.y)
            w2c_1_li.append(w2c_1.rotation.z)
            w2c_1_li.append(w2c_1.rotation.w)
            # w2c_2_ls = []
            # w2c_2_ls.append(w2c_2.rotation.x)
            # w2c_2_ls.append(w2c_2.rotation.y)
            # w2c_2_ls.append(w2c_2.rotation.z)
            # w2c_2_ls.append(w2c_2.rotation.w)            
            
            t1_matrix = tf_transformations.quaternion_matrix(w2c_1_li) # 已经是numpy数组了
            # print(t1_matrix)
            # w2c_2_matrix = tf_transformations.quaternion_matrix(w2c_2_ls)
            # tf_transformations.translation_matrix() 
            # 可以直接将平移向量转化为齐次矩阵，平移齐次矩阵乘以旋转齐次矩阵就得到了最终的齐次变换矩阵
            # 因为这种方法还要先建列表，也并不多简单，因此直接采取下面的方法修改
            t1_matrix[0][3] = w2c_1.translation.x
            t1_matrix[1][3] = w2c_1.translation.y
            t1_matrix[2][3] = w2c_1.translation.z
            pose_matrix_t=np.array([w2c_1.translation.x,w2c_1.translation.y,w2c_1.translation.z])
            pose_matrix=np.reshape(pose_matrix_t,(-1,1))
            # w2c_2_matrix[0][3] = w2c_2.translation.x
            # w2c_2_matrix[1][3] = w2c_2.translation.y
            # w2c_2_matrix[2][3] = w2c_2.translation.z
            # paraM=np.matmul(inmatrix1,t1_matrix)
            t1_matrix_inv=np.linalg.inv(t1_matrix)
            # h_to_div=(msg.u1-msg.u2)*t1_matrix[2][0]/inmatrix[0][0]+\
            #             (msg.v1-msg.v2)*t1_matrix[2][1]/inmatrix[1][1]
            h_to_div=(msg.u1-msg.u2)*t1_matrix_inv[2][0]/inmatrix[0][0]+\
                        (msg.v1-msg.v2)*t1_matrix_inv[2][1]/inmatrix[1][1]
            if(h_to_div<0.1):
                print("数据有误")
            else:
                depth=person_height/h_to_div
                # print("深度：%.3f" %depth)
                #depth即zc
                mid_u=(msg.u1+msg.u2)/2
                mid_v=msg.v1
                print("mid_u: %.3f" % mid_u)
                print("mid_v: %.3f" % mid_v)
                print("机械臂位姿：")
                print(pose_matrix_t)
                x_mid=(mid_u-inmatrix[0][2])*depth/inmatrix[0][0] #x_high=(u_high-cx)*depth/fx
                y_mid=(mid_v-inmatrix[1][2])*depth/inmatrix[1][1] #y_high=(v_high-cy)*depth/fy
                # print("x_mid: %.3f" % x_mid)
                # print("y_mid: %.3f" % y_mid)
                x_mid=(mid_u-inmatrix[0][2])*depth/inmatrix[0][0] #x_high=(u_high-cx)*depth/fx
                camera_high_coordinate_t = np.array([x_mid,y_mid,depth])
                # camera_high_coordinate_t = np.array([x_high,y_high,depth,1.0])
                # print(camera_high_coordinate_t)
                camera_high_coordinate = np.reshape(camera_high_coordinate_t,(-1,1))
                # print("相机坐标：")
                # print(camera_high_coordinate)
                # t1_matrix_inv=np.linalg.inv(t1_matrix)
                
                # world_z_high = x_high*t1_matrix_inv[2][0]+y_high*t1_matrix_inv[2][1]+depth*t1_matrix_inv[2][2]+t1_matrix_inv[2][3]
                # world_z_low = x_low*t1_matrix_inv[2][0]+y_low*t1_matrix_inv[2][1]+depth*t1_matrix_inv[2][2]+t1_matrix_inv[2][3]
                # print(world_z_high)
                # print(world_z_low)
                rota_matrix = np.array([[t1_matrix[0][0],t1_matrix[0][1],t1_matrix[0][2]],\
                                        [t1_matrix[1][0],t1_matrix[1][1],t1_matrix[1][2]],\
                                        [t1_matrix[2][0],t1_matrix[2][1],t1_matrix[2][2]]])
                rota_matrix_inv = np.linalg.inv(rota_matrix)
                world_high_coordinate = np.matmul(rota_matrix_inv,camera_high_coordinate)-np.matmul(rota_matrix_inv,pose_matrix)
                print(world_high_coordinate)
                # print("ratio_u: %.2f" %abs(msg.u2-msg.u1))
                # print(world_vector[0])
                
                world_x=world_high_coordinate[0][0]
                world_y=world_high_coordinate[1][0]
                world_z=world_high_coordinate[2][0]
                # self.swing_flag=object_world_msg.move_flag
                # angle_offset=math.degrees(object_world_msg.cam2person_angle)
                # self.angle_offset=angle_offset
                # p2w_angle=math.atan2(object_world_msg.x,-object_world_msg.y)
                # offset_x=0.16*math.tan(0.3*p2w_angle)
                # c2w_angle=math.atan2(object_world_msg.x-offset_x,-(object_world_msg.y+0.16))

                # if(world_z>1.55 and world_z<1.75):
                self.to_pub.header.stamp = self.get_clock().now().to_msg()
                self.to_pub.object_world_x=world_x
                self.to_pub.object_world_y=world_y
                self.to_pub.object_world_z=world_z
                # p2w_angle=math.atan2(world_x,-world_y)
                # offset_x=0.16*math.tan(0.3*p2w_angle)
                # offset_x=0.0
                self.to_pub.offset_x=depth
                self.to_pub.cam2person_angle=math.atan2(world_x,-(world_y+0.16))
                self.to_pub.move_flag=True
                # print(math.degrees(math.atan2(world_x,-(world_y+0.16))))
                self.object_world_pub.publish(self.to_pub)
                # camera_u,camera_v,camera_w=self.convert_depth_to_phys_coord_using_realsense(msg.u1, msg.v1, depth, inmatrix, dist_param=distort_param)
                # print(camera_u,camera_v,camera_w)

    def convert_depth_to_phys_coord_using_realsense(self,x, y, depth, K,dist_param):
        '''
        给出像素坐标，深度，内参矩阵，畸变参数求相机坐标
        '''
        _intrinsics = rs.intrinsics()
        _intrinsics.width = 640
        _intrinsics.height = 480
        _intrinsics.ppx =K[0][2]
        _intrinsics.ppy =K[1][2]
        _intrinsics.fx =K[0][0]
        _intrinsics.fy =K[1][1]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = rs.distortion.none
        _intrinsics.coeffs = dist_param
        result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        #result[0]: right, result[1]: down, result[2]: forward
        return result[0],result[1],result[2]

def main(args=None):
    rclpy.init(args=args)

    worldVectorPuber = WorldVectorPub()

    rclpy.spin(worldVectorPuber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    worldVectorPuber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
