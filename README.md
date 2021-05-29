```
git clone https://github.com/uptopia/save_realsense_data.git
cd <path>/save_realsense_datasrc/python_save_realsense_data/src/
chmod u+x python_save_realsense_data.py 
cd <path>/save_realsense_data
catkin_make
```

```
<Terminal 1>
    $roscore

<Terminal 2> Realsense D435i
    $cd <realsense-ros_ws>

    [ROS Topic: /camera/depth/color/points]
    $roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

    [ROS Topic: /camera/depth_registered/points]
    $roslaunch realsense2_camera rs_rgbd.launch

    [ROS Topic: /camera/depth_registered/points; /camera/depth/color/points]
    $roslaunch realsense2_camera rs_rgbd.launch filters:=pointcloud
 
<Terminal 3> [Python]
    $cd <path>/save_realsense_data
    $. devel/setup.bash
    $rosrun python_save_realsense_data python_save_realsense_data.py
OR
<Terminal 3> [C++]
    $cd <path>/save_realsense_data
    $. devel/setup.bash
    $rosrun cpp_save_realsense_data cpp_save_realsense_data
```

![Python 2.7.17](https://img.shields.io/badge/python-2.7.17-green.svg)
![Python 3.6.9](https://img.shields.io/badge/python-3.6.9-green.svg) (TODO)
# Show and Sav Realsense Data (Python, C++)

## Dependencies
* Install [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

## Software environment
* Ubuntu 18.04
* ROS Melodic
* Python 3.6.9
* opencv 4.5.1 cv2
* cv_bridge (python2 不用)(python3 待測試)
* ROS Melodic

## cpp_save_realsense_data
* 訂閱ROS Topic: `/camera/depth_registered/points`
=>有序點雲 Organized Point Cloud <br><br/>

* 訂閱ROS Topic: `/camera/depth/color/points`
=>無序點雲 Unorganized Point Cloud (height = 1)<br><br/>


## python_save_realsense_data
* 訂閱ROS Topic: `/camera/aligned_depth_to_color/image_raw`
=>深度圖 Depth image <br><br/>

* 訂閱ROS Topic: `/camera/depth_registered/points`
=>有序點雲 Organized Point Cloud <br><br/>

* 訂閱ROS Topic: `/camera/color/image_raw`
=>RGB圖 RGB image <br><br/>
