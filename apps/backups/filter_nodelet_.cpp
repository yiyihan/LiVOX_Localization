#include <mutex>
#include <memory>
#include <iostream>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
 

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/registration/correspondence_estimation.h>

#include <pclomp/ndt_omp.h>

#include <hdl_localization/pose_estimator.hpp>

using namespace std;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

namespace hdl_localization {

class FilteresultServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  FilteresultServerNodelet() {
  }
  virtual ~FilteresultServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();
   
    // publish globalmap with "latched" publisher
    points_sub = mt_nh.subscribe("/livox_map", 1, &FilteresultServerNodelet::points_callback, this);
  }



private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */



  void points_callback(const sensor_msgs::PointCloud2ConstPtr& livox_msg) {

    NODELET_INFO("all received!");
    const auto& stamp = livox_msg->header.stamp;

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());  
    pcl::PointCloud<PointT>::Ptr pcl_cloud_trans(new pcl::PointCloud<PointT>()); 
    // pcl::PointCloud<PointT>::Ptr pcl_cloud_trans_v(new pcl::PointCloud<PointT>()); 

    pcl::fromROSMsg(*livox_msg, *pcl_cloud_trans);

    
  }

  private:
  // ROS
  
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::Subscriber points_sub;
  // message_filters::Subscriber points_sub_; 
  ros::Subscriber globalmap_sub;
  ros::Publisher filteresult_pub;

  pcl::PointCloud<PointT>::Ptr velodynemap;
  pcl::PointCloud<PointT>::Ptr globalmap;

  tf::TransformBroadcaster pose_broadcaster;
  tf::TransformListener tf_listener;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::FilteresultServerNodelet, nodelet::Nodelet)
