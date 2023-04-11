// #include <mutex>
// #include <memory>
// #include <iostream>

// #include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <tf_conversions/tf_eigen.h>
// #include <tf/transform_broadcaster.h>

// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>

// #include <nodelet/nodelet.h>
// #include <pluginlib/class_list_macros.h>

// #include <pcl/filters/voxel_grid.h>

// #include <pclomp/ndt_omp.h>

// #include <hdl_localization/pose_estimator.hpp>


// namespace hdl_localization {

// class GlobalmapServerNodelet : public nodelet::Nodelet {
// public:
//   using PointT = pcl::PointXYZI;

//   GlobalmapServerNodelet() {
//   }
//   virtual ~GlobalmapServerNodelet() {
//   }

//   void onInit() override {
//     nh = getNodeHandle();
//     mt_nh = getMTNodeHandle();
//     private_nh = getPrivateNodeHandle();

//     initialize_params();

//     // publish globalmap with "latched" publisher
//     globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
//     globalmap_pub.publish(globalmap);
//   }

// private:
//   void initialize_params() {
//     // read globalmap from a pcd file
//     std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
    
//     globalmap.reset(new pcl::PointCloud<PointT>());
//     pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
//     globalmap->header.frame_id = "map";

//     // downsample globalmap
//     double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
//     boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
//     voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
//     voxelgrid->setInputCloud(globalmap);

//     pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
//     voxelgrid->filter(*filtered);

//     globalmap = filtered;
//   }

// private:
//   // ROS
//   ros::NodeHandle nh;
//   ros::NodeHandle mt_nh;
//   ros::NodeHandle private_nh;

//   ros::Publisher globalmap_pub;

//   pcl::PointCloud<PointT>::Ptr globalmap;
// };

// }


// PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)








#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>


namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;
  using PointType = pcl::PointXYZL ;
  GlobalmapServerNodelet() {
  }
  virtual ~GlobalmapServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    // publish globalmap with "latched" publisher
    globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    roadplane_pub = nh.advertise<sensor_msgs::PointCloud2>("/roadplane", 5, true);
    globalmap_pub.publish(globalmap);
    roadplane_pub.publish(roadplane);
  }

private:
  void initialize_params() {
    // read globalmap from a pcd file
    std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");

    std::string roadplane_pcd = private_nh.param<std::string>("roadplane_pcd", "");
    // std::cout<<"global map path: "<<globalmap_pcd<<std::endl;
    globalmap.reset(new pcl::PointCloud<PointType>());
    roadplane.reset(new pcl::PointCloud<PointType>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    pcl::io::loadPCDFile(roadplane_pcd, *roadplane);
    globalmap->header.frame_id = "map";
    roadplane->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    boost::shared_ptr<pcl::VoxelGrid<PointType>> voxelgrid(new pcl::VoxelGrid<PointType>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;

    boost::shared_ptr<pcl::VoxelGrid<PointType>> Voxelgrid(new pcl::VoxelGrid<PointType>());
    Voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    Voxelgrid->setInputCloud(roadplane);

    pcl::PointCloud<PointType>::Ptr filtered_roadplane(new pcl::PointCloud<PointType>());
    Voxelgrid->filter(*filtered_roadplane);

    roadplane = filtered_roadplane;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Publisher roadplane_pub;
  pcl::PointCloud<PointType>::Ptr globalmap;
  pcl::PointCloud<PointType>::Ptr roadplane;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)

