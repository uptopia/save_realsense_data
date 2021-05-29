# -*- coding: utf-8 -*- #
# =====================================
#        Stream Realsense Data
# (Depth, RGB,  Organized Point Cloud)
# =====================================
# written by Shang-Wen, Wong. (2021.4.20)

'''
Realsense D435i
$roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud
'''

import rospy

import ctypes
import struct

import numpy as np

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2, PointField

import message_filters

import cv2

import sys
if sys.version > '3': #ROS自帶的cv_bridge只支持python2，想要使用Python3需要自行編譯cv_bridge包
    sys.path.insert(0, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')
from cv_bridge import CvBridge, CvBridgeError


def image_rgb_cb(data_rgb):
    print("image_rgb_cb")
    global rgb_image
    
    bridge = CvBridge()
    try:
        #讀取RGB圖(dtype = np.uint8)
        rgb_image = bridge.imgmsg_to_cv2(data_rgb, "bgr8") #data_rgb.encoding) #
        # w, h, z = np.shape(rgb_image)
        # print("rgb_image w, h, z = {0}, {1}, {2}".format(w, h, z))
        # print("rgb_image.dtype", rgb_image.dtype)

        # cv2.imshow('rgb_image', rgb_image)
        # cv2.waitKey(1)        

    except CvBridgeError as e:
        print(e)


def image_depth_cb(data_depth):
    print("image_depth_cb")
    global depth_image
    global depth_3channel
    
    bridge = CvBridge()
    try:
        #讀取深度圖(dtype = np.float32)
        depth_image = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
        # print('data_depth.encoding', data_depth.encoding)
        # w, h = np.shape(depth_image)
        # print("depth_image w, h = {0}, {1}".format(w, h))
        # print("depth_image.dtype", depth_image.dtype)

        #深度圖正規化到[0, 1]，再縮放＊255
        depth_image = np.array(depth_image, dtype = np.float32)
        cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX) 
        depth_image *= 255

        #單通道轉換成三通道
        height = np.shape(depth_image)[0]
        width = np.shape(depth_image)[1]
        depth_3channel = np.zeros((height, width, 3), dtype = np.uint8)
        depth_3channel[:,:,0] = depth_image
        depth_3channel[:,:,1] = depth_image
        depth_3channel[:,:,2] = depth_image        
        # w, h, z = np.shape(depth_3channel)
        # print("depth3channel_image w, h, z = {0}, {1}, {2}".format(w, h, z))
        # print("depth_3channel.dtype", depth_3channel.dtype)

        # cv2.imshow('depth_3channel', depth_3channel)
        # cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)


def cloud_cb(data_cloud):
    global cloud_image

    print("cloud_cb")

    # [ROS sensor_msg]PointCloud2點雲資訊
    # print('width', data_cloud.width) #640
    # print('height', data_cloud.height) #480
    # print('header', data_cloud.header) #coordinate frame ID
    # print('fields', data_cloud.fields) #x, y, z, rgb
    # print("is_dense", data_cloud.is_dense) #True: no nan; False: have nan    

    # 初始化
    width = data_cloud.width
    height = data_cloud.height
    points_list = np.empty([height, width, 6], dtype = np.float32)
    cloud_image = np.zeros((height, width, 3), dtype = np.uint8)

    # 讀取有序點雲資料並轉成圖
    m = 0
    n = 0
    gen = pc2.read_points(data_cloud, skip_nans = False)  #"有序點雲": skip_nans = False
    int_data = list(gen)
    for x in int_data:

        # 將rgb(float32)格式轉成r, g, b(int)格式
        rgb = x[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,rgb)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)

        points_list[m, n] = [x[0], x[1], x[2], r, g, b]
        cloud_image[m, n] = (b, g, r)

        if(n == width - 1):
            m += 1
            n = 0
        else:
            n += 1

        if(m == height):
            break

    # w, h, z = np.shape(cloud_image)
    # print("cloud_image w, h, z = {0}, {1}, {2}".format(w, h, z))
    # cv2.imshow('cloud_image', cloud_image)
    # cv2.waitKey(1)    


def DisplayImage():
    global depth_3channel, rgb_image, cloud_image

    w0, h0, z0 = np.shape(depth_3channel)
    w1, h1, z1 = np.shape(cloud_image)
    w2, h2, z2 = np.shape(rgb_image)
    print("(depth_3channel) w, h, z, dtype = {0}, {1}, {2}, {3}".format(w0, h0, z0, depth_3channel.dtype))
    print("(cloud_image) w, h, z, dtype = {0}, {1}, {2}, {3}".format(w1, h1, z1, cloud_image.dtype))
    print("(rgb_image) w, h, z, dtype = {0}, {1}, {2}, {3}".format(w2, h2, z2, rgb_image.dtype))

    img_hstack = np.hstack([depth_3channel, cloud_image, rgb_image])
    cv2.imshow('[TOP_深度]/camera/aligned_depth_to_color/image_raw; [MID_有序點雲RGB]/camera/depth_registered/points; [BTM_RGB]/camera/color/image_raw', img_hstack)
    cv2.waitKey(1)    


def timesync_cb(data_depth, data_cloud, data_rgb):
    print("timesync_cb")

    cloud_cb(data_cloud)
    image_depth_cb(data_depth)
    image_rgb_cb(data_rgb)

    # 同步顯示[深度][有序點雲RGB][RGB]張圖
    DisplayImage()

if __name__ == '__main__':

    rospy.init_node("get_organized_cloud")

    # 訂閱Realsense ROS Topics: $roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud
    sub_depth = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image)   # 深度圖
    sub_cloud = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)          # 有序點雲
    sub_rgb = message_filters.Subscriber('/camera/color/image_raw', msg_Image)                      # RGB圖

    # ROS資料同步
    ts = message_filters.TimeSynchronizer([sub_depth, sub_cloud, sub_rgb], 10)
    ts.registerCallback(timesync_cb)

    rospy.spin()

# Reference:
# PointCloud2格式：http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
# 點雲轉成numpy：https://answers.ros.org/question/344096/subscribe-pointcloud-and-convert-it-to-numpy-in-python/
