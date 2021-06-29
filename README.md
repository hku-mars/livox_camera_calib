# livox_camera_calib
**livox_camera_calib** is a robust, high accuracy extrinsic calibration tool between high resolution LiDAR (e.g. Livox) and camera in targetless environment. Our algorithm can run in both indoor and outdoor scenes, and only requires edge information in the scene. If the scene is suitable, we can achieve pixel-level accuracy similar to or even beyond the target based method.
<div align="center">
    <img src="pics/color_cloud.png" width = 100% >
    <font color=#a0a0a0 size=2>An example of a outdoor calibration scenario. We color the point cloud with the calibrated extrinsic and compare with actual image. A and C are locally enlarged
views of the point cloud. B and D are parts of the camera image
corresponding to point cloud in A and C.</font>
</div>

## Info
New features in next version:
1. Support spinning LiDAR
2. Support muti-scenes calibration (more accuracy)

## Related paper
Related paper available on arxiv:  
[Pixel-level Extrinsic Self Calibration of High Resolution LiDAR and Camera in Targetless Environments](http://arxiv.org/abs/2103.01627)
## Related video
Related video: https://youtu.be/e6Vkkasc4JI

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

```
    sudo apt-get install ros-XXX-cv-bridge ros-xxx-pcl-conversions
```

### 1.2 **Eigen**
Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### 1.3 **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.4 **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html). (Our code is tested with PCL1.7)

## 2. Build
Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/livox_camera_calib.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 3. Run our example
Download [Our recorded rosbag](https://drive.google.com/drive/folders/1pBvE_nrg60IUo7PXDRsbBwDI68sq5LS6?usp=sharing) to your local path, and then change the path in **calib.launch** to your data path. Then directly run
```
roslaunch livox_camera_calib calib.launch
```
If you have trouble in downloading the rosbag files, you can download the same files from Baidu net-disk.
```
Link (链接): https://pan.baidu.com/s/197hsjmO42p5OIUjo_l4kkg 
Extraction code (提取码): myfm
```
## 4. Run on your own sensor set
### 4.1 Record data
Record the point cloud and image msg to rosbag (15s or more for avia LiDAR). Then change the topic name in **config_outdoor.yaml** file
```
# Topic name in rosbag
PointCloudTopic: "/livox/lidar"
ImageTopic: "/camera/color/image_raw"
```
### 4.2 Modify some params
Modify the camera matrix and distortion coeffs in **camera.yaml**  
Modify the initial extrinsic in **config_outdoor.yaml** if needed.
