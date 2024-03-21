'''
使用临近插值去畸变
'''


import cv2
import math
import numpy as np
def main():
    k1 = -0.28340811
    k2 = 0.07395907
    p1 = 0.00019359
    p2 = 1.76187114e-05
    fx = 458.654
    fy = 457.296
    cx = 367.215
    cy = 248.375
    inmatrix = np.array([[fx,0.000000,cx],[0.000000,fy,cy],[0.000000,0.000000,1.000000]])
    distort_param = np.array([k1,k2,p1,p2,0.000000])
    
    filepath = '/home/mwq/bringup_ws/src/world_vector_pub/world_vector_pub/distorted.png'
    image = cv2.imread(filepath,0)
    # image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    print(image.shape)
    print(type(image))
    img_size = image.shape
    rows = img_size[0]#行数，即宽度
    cols = img_size[1]#列数，即长度
    map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix=inmatrix, distCoeffs=distort_param, R=None, newCameraMatrix=None, size=(cols,rows), m1type=cv2.CV_32FC1)
    print(map1.shape)
    print(map2.shape)

    image_undistort = []
    image_undistort = np.zeros((rows,cols),dtype=np.uint8)
    mymap1 = np.zeros((rows,cols),dtype=np.uint16)
    mymap2 = np.zeros((rows,cols),dtype=np.uint16)
    # print(image_undistort)
    print(type(image_undistort))
    print(image_undistort.shape)
    for v in range(rows):
        for u in range(cols):
            x = (u - cx) / fx
            y = (v - cy) / fx
            r = math.sqrt(x * x + y * y)
            x_distorted = x*(1+k1*r*r+k2*r*r*r*r)+2*p1*x*y+p2*(r*r+2*x*x)
            y_distorted = y*(1+k1*r*r+k2*r*r*r*r)+2*p2*x*y+p1*(r*r+2*y*y)
            u_distorted = fx * x_distorted + cx
            v_distorted = fy * y_distorted + cy
            u_dint=int(u_distorted)
            v_dint=int(v_distorted)
            mymap1[v_dint][u_dint]=u
            mymap2[v_dint][u_dint]=v
            # print(v_dint,map2[v_dint][0],v)
            # if ((u >= 0)and (v >=0) and (u< cols) and (v < rows)):
            #     image_undistort[v][u]=image[v_dint][u_dint]
            # else:
            #     image_undistort[undistort_u][undistort_v]=0
    
    for v in range(rows):
        for u in range(cols):
                if ((u >= 0)and (v >=0) and (u< cols) and (v < rows)):
                    v_undistorted=mymap2[v][u]
                    # print(v_undistorted)
                    u_undistorted=mymap1[v][u]
                    # print(u_undistorted)
                    image_undistort[v_undistorted][u_undistorted]=image[v][u]
    
    
    while True:
        cv2.imshow("distorted",image_undistort)
    #加入cv2.getWindowProperty()的窗口函数判断，当窗口关闭时返回-1，结束循环，使得程序可以自动结束
        if cv2.getWindowProperty('distorted', 0) == -1: 
            break
        cv2.waitKey(1)
    #调整窗口大小
    cv2.destroyAllWindows()
       
    
if __name__ == '__main__':
    main()
