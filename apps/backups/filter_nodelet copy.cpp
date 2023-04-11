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
#include <message_filters/sync_policies/approximate_time.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

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

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

namespace hdl_localization {

class FilteresultServerNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;
  using PointType = pcl::PointXYZL;
  FilteresultServerNodelet() {
  }
  virtual ~FilteresultServerNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();
    
    points_sub.subscribe(n_, "/painted_pointcloud_wL", 1);
    trans_sub.subscribe(n_, "/odom", 1);

    sync_.reset(new sync( SyncPolicy(10), points_sub, trans_sub));
    sync_->registerCallback(boost::bind(&FilteresultServerNodelet::points_callback,this, _1, _2));

    filteresult_pub = nh.advertise<sensor_msgs::PointCloud2>("/filter_result", 5, true);
  }


  void points_callback(const sensor_msgs::PointCloud2ConstPtr& livox_msg, const nav_msgs::OdometryConstPtr& odom_msg) {

    // NODELET_INFO("all received!");
    const auto& stamp = livox_msg->header.stamp;

    pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>());  
    pcl::PointCloud<PointType>::Ptr pcl_cloud_trans(new pcl::PointCloud<PointType>()); 
    geometry_msgs::Quaternion odom_quat;

    sensor_msgs::PointCloud2 labeled_pointcloud;

    Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf quat;

    pose_(0, 3) = odom_msg->pose.pose.position.x ;
    pose_(1, 3) = odom_msg->pose.pose.position.y ;
    pose_(2, 3) = odom_msg->pose.pose.position.z ;
    odom_quat = odom_msg->pose.pose.orientation ;

    quat.w() = odom_quat.w ;
    quat.x() = odom_quat.x ;
    quat.y() = odom_quat.y ;
    quat.z() = odom_quat.z ;

    // Eigen::Matrix3f R = quat.normalized().toRotationMatrix();
    pose_.block<3, 3>(0, 0) = quat.normalized().toRotationMatrix();

    pcl::fromROSMsg(*livox_msg, *pcl_cloud);
    pcl::transformPointCloud (*pcl_cloud, *pcl_cloud_trans, pose_);

    pcl::toROSMsg(*pcl_cloud_trans, labeled_pointcloud);

    labeled_pointcloud.header.frame_id = "map";
    filteresult_pub.publish(labeled_pointcloud) ;

    std::cout<<"the labeled pointcloud : "<< pcl_cloud_trans->points[0].label<<std::endl;
  }

  private:
  // ROS
  
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::NodeHandle n_;

  ros::Publisher filteresult_pub;
 
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub;
  message_filters::Subscriber<nav_msgs::Odometry> trans_sub ;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> sync;
  boost::shared_ptr<sync> sync_;

  pcl::PointCloud<PointT>::Ptr velodynemap;
  pcl::PointCloud<PointT>::Ptr globalmap;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::FilteresultServerNodelet, nodelet::Nodelet)
