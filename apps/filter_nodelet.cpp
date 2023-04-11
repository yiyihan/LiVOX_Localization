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
    
    points_sub.subscribe(n_, "/velodyne_points", 1);
    labeled_sub = n_.subscribe("/painted_pointcloud_wL", 1, &FilteresultServerNodelet::aligned_callback, this);

    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(points_sub, tf_, "/map", 1);
    // sync_.reset(new sync( SyncPolicy(10), points_sub, trans_sub));
    tf_filter_->registerCallback(boost::bind(&FilteresultServerNodelet::points_callback,this, _1));

    filteresult_pub = nh.advertise<sensor_msgs::PointCloud2>("/filter_result", 5, true);
  }


  void aligned_callback(const sensor_msgs::PointCloud2ConstPtr& labeled_pointcloud_msg) {

    NODELET_INFO("labeled pointcloud received!");
    
    pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>());  
    pcl::PointCloud<PointType>::Ptr pcl_cloud_trans(new pcl::PointCloud<PointType>()); 
    sensor_msgs::PointCloud2 labeled_pointcloud;

    pcl::fromROSMsg(*labeled_pointcloud_msg, *pcl_cloud);
    pcl::transformPointCloud (*pcl_cloud, *pcl_cloud_trans, transform_to_map);
    pcl::toROSMsg(*pcl_cloud_trans, labeled_pointcloud);

    labeled_pointcloud.header.frame_id = "map";
    filteresult_pub.publish(labeled_pointcloud);

    std::cout<<"the labeled pointcloud : "<< pcl_cloud_trans->points[0].label<<std::endl;

  }
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& livox_msg) {

    NODELET_INFO("all received!");
    const auto& stamp = livox_msg->header.stamp;

    if(tf_.canTransform("/map", "/velodyne", ros::Time(0))){
        
        tf_.lookupTransform("/map", "/velodyne", ros::Time(0), transform); 

    }
    pcl_ros::transformAsMatrix(transform, transform_to_map);

  }

  private:
  // ROS
  
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::NodeHandle n_;

  ros::Subscriber labeled_sub ;
  ros::Publisher filteresult_pub;
 
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub;

  tf::TransformListener tf_;
  tf::StampedTransform transform;
  tf::MessageFilter<sensor_msgs::PointCloud2> * tf_filter_;

  pcl::PointCloud<PointT>::Ptr velodynemap;
  pcl::PointCloud<PointT>::Ptr globalmap;

  Eigen::Matrix4f transform_to_map ;
};

}


PLUGINLIB_EXPORT_CLASS(hdl_localization::FilteresultServerNodelet, nodelet::Nodelet)
