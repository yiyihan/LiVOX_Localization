# hdl_localization

***hdl_localization*** is a ROS package for real-time 3D localization using a 3D LIDAR, such as velodyne HDL32e and VLP16. This package performs Unscented Kalman Filter-based pose estimation. It first estimates the sensor pose from IMU data implemented on the LIDAR, and then performs multi-threaded NDT scan matching between a globalmap point cloud and input point clouds to correct the estimated pose. IMU-based pose prediction is optional. If you disable it, the system predicts the sensor pose with the constant velocity model without IMU information.

We reimplement it on our own dataset with 3d object detection on pre-built map of our previous [repo](https://github.com/yiyihan/Map_Render). 

We work on 3D Object Detection and HDL-Localization for visualizing detected objects on pointcloud maps. Our sensors include

Real-world testing:
![](pic/demo.gif)

Our Sensor Platform:
![](pic/Image.jpeg)

## Requirements
***hdl_localization*** requires the following libraries:
- OpenMP
- PCL 1.7

The following ros packages are required:
- pcl_ros
- <a href="https://github.com/koide3/ndt_omp">ndt_omp</a>

## Parameters
All parameters are listed in *launch/hdl_localization.launch* as ros params.<br>
You can specify the initial sensor pose using "2D Pose Estimate" on rviz, or using ros params (see example launch file).



```bash
rosparam set use_sim_time true
roslaunch hdl_localization hdl_localization.launch
```

```bash
roscd hdl_localization/rviz
rviz -d hdl_localization.rviz
```

```bash
rosbag play --clock XXX.bag
```


