#include <mutex>
#include <memory>
#include <iostream>
#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pclomp/ndt_omp.h>
#include <pcl/search/kdtree.h>
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/io.h>
#include<pcl/registration/correspondence_estimation.h>
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/io.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/registration/correspondence_estimation.h>
using namespace std;
using namespace message_filters;
#include <hdl_localization/pose_estimator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using PointT = pcl::PointXYZI;
using PointType = pcl::PointXYZL;
pcl::PointCloud<PointT>::Ptr laserCloudFullRes2(new pcl::PointCloud<PointT>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<PointT>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointType>::Ptr laserCloudRoadResColor_pcd(new pcl::PointCloud<PointType>());


std::string map_file_path = "/home/yihan/relocalization/src";
std::string all_points_filename(map_file_path + "/all_points.pcd");


namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  

  HdlLocalizationNodelet() {
  }
  virtual ~HdlLocalizationNodelet() {
  }

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    processing_time.resize(16);
    initialize_params();
    
    odom_child_frame_id = private_nh.param<std::string>("odom_child_frame_id", "base_link");
    
    use_imu = private_nh.param<bool>("use_imu", true);
    invert_imu = private_nh.param<bool>("invert_imu", false);
    if(use_imu) {
      NODELET_INFO("enable imu-based prediction");
      imu_sub = mt_nh.subscribe("/gpsimu_driver/imu_data", 256, &HdlLocalizationNodelet::imu_callback, this);
    }
    points_sub = mt_nh.subscribe("/velodyne_points", 1, &HdlLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmap_callback, this);
    livox_sub = mt_nh.subscribe("/livox/lidar", 1, &HdlLocalizationNodelet::points_callback_, this);
    // livox_sub = mt_nh.subscribe("/painted_pointcloud_wL", 1, &HdlLocalizationNodelet::points_callback_, this);
    painted_cloud_sub = mt_nh.subscribe("/filtered_labeled_pointcloud", 1, &HdlLocalizationNodelet::Points_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 1, &HdlLocalizationNodelet::initialpose_callback, this);

    pose_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 1, false);

    filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_livox", 5, false);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox_map", 1, false);


  }

private:
  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");

    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    // double threshold = private_nh.param<double>("threshold", 4.0);

    pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if(ndt_neighbor_search_method == "DIRECT1") {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else if(ndt_neighbor_search_method == "DIRECT7") {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else {
      if(ndt_neighbor_search_method == "KDTREE") {
        NODELET_INFO("search_method KDTREE is selected");
      } else {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    registration = ndt;

    // initialize pose estimator
    if(private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(registration,
        ros::Time::now(),
        Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0), private_nh.param<double>("init_ori_x", 0.0), private_nh.param<double>("init_ori_y", 0.0), private_nh.param<double>("init_ori_z", 0.0)),
        private_nh.param<double>("cool_time_duration", 0.5)
      ));
    }
  }

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imu_callback(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }
  void points_callback_ (const sensor_msgs::PointCloud2ConstPtr& point_msgs_livox){

    
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);

    const auto& stamp = point_msgs_livox->header.stamp;

    NODELET_INFO("RECEIVED livox ^ ^");
    pcl::PointCloud<PointT>::Ptr pcl_cloud_livox(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pcl_cloud_livox_(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr laserCloudFullRes(new pcl::PointCloud<PointT>());

    // pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    laserCloudFullRes->clear();
    laserCloudFullResColor->clear();
    pcl::fromROSMsg(*point_msgs_livox, *pcl_cloud_livox);
    pcl::fromROSMsg(*point_msgs_livox, *laserCloudFullRes);

    // velo2livox
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    transform_1(0,0) = 9.99997456e-01 ;
    transform_1(0,1) = -2.18275887e-03  ;
    transform_1(0,2) = 3.93688199e-04 ;
    transform_1(0,3) = -6.25288242e-02 ;
    transform_1(1,0) = 2.18272349e-03  ;
    transform_1(1,1) = 9.99997762e-01 ;
    transform_1(1,2) =  3.55326510e-05 ;
    transform_1(1,3) = -2.44046831e-02  ;
    transform_1(2,0) = -3.93765045e-04 ;
    transform_1(2,1) = -3.46785103e-05  ;
    transform_1(2,2) = 9.99999878e-01  ;
    transform_1(2,3) = 1.65635988e-01 ;
    // velo2map
    // std::cout<<"transofrm is : "<<transform_to_map<<std::endl;
    // velo2livox
    // std::cout<<"transofrm_1 inverse is : "<<transform_1.inverse()<<std::endl;

    // std::cout<<"transofrm_1 is : "<<transform_1<<std::endl;
    pcl::transformPointCloud (*pcl_cloud_livox, *pcl_cloud_livox_, transform_to_map *transform_1.inverse());

    auto filtered_livox = downsample(pcl_cloud_livox);
    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    } 
    else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(stamp < (*imu_iter)->header.stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double gyro_sign = invert_imu ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

      // correct
      auto t1 = ros::WallTime::now();
      auto aligned_livox_ = pose_estimator->correct(filtered_livox);
      auto t2 = ros::WallTime::now();

      processing_time.push_back((t2 - t1).toSec());
      double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();

      sensor_msgs::PointCloud2 aligned_livox;
      sensor_msgs::PointCloud2 laserCloudFullRes3;

      laserCloudFullRes2->clear();
      *laserCloudFullRes2 = *aligned_livox_;

      int laserCloudFullResNum = laserCloudFullRes2->points.size();
      for (int i = 0; i < laserCloudFullResNum; i++) {

          pcl::PointXYZRGB temp_point;
          RGBpointAssociateToMap(&laserCloudFullRes2->points[i], &temp_point);
         
          laserCloudFullResColor->push_back(temp_point);
      }

      // *laserCloudFullResColor_pcd += *laserCloudFullResColor;
      *laserCloudFullResColor_pcd += *laserCloudFullRes2;

      pcl::toROSMsg(*laserCloudFullResColor, laserCloudFullRes3);
      
      pcl::toROSMsg(*aligned_livox_, aligned_livox);

      aligned_livox.header.frame_id = "map";
      laserCloudFullRes3.header.frame_id = "map";
      map_pub.publish(laserCloudFullRes3);
      filter_pub.publish(aligned_livox);


  }

 

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      NODELET_ERROR("waiting for initial pose input!!");
      return;
    }

    if(!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud_trans(new pcl::PointCloud<PointT>());

    // pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud_trans);
    if(pcl_cloud_trans->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    
    // pcl::transformPointCloud (*pcl_cloud_trans, *pcl_cloud, transform_1.inverse());
    // transform pointcloud into odom_child_frame_id  
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());  

    if(tf_listener.canTransform("/map", "/velodyne", ros::Time(0))){

        tf_listener.lookupTransform("/map", "/velodyne", ros::Time(0), transform); 
    }
    //  transform velodyne to map matrix
    pcl_ros::transformAsMatrix(transform, transform_to_map);
    std::cerr<<"x of transform: "<< transform_to_map<<std::endl;
    if(!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud_trans, *cloud, this->tf_listener)) {
        
        NODELET_ERROR("point cloud cannot be transformed into target frame!!");
        return;
    } 

    auto filtered = downsample(cloud);

    // predict
    if(!use_imu) {
      pose_estimator->predict(stamp, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      auto imu_iter = imu_data.begin();
      for(imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if(stamp < (*imu_iter)->header.stamp) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double gyro_sign = invert_imu ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // correct
    auto t1 = ros::WallTime::now();
    auto aligned = pose_estimator->correct(filtered);
    auto t2 = ros::WallTime::now();

    processing_time.push_back((t2 - t1).toSec());
    double avg_processing_time = std::accumulate(processing_time.begin(), processing_time.end(), 0.0) / processing_time.size();
    // NODELET_INFO_STREAM("processing_time: " << avg_processing_time * 1000.0 << "[msec]");

    if(aligned_pub.getNumSubscribers()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      aligned_pub.publish(aligned);
    }
    
    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());
  }


void Points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg){

  pcl::fromROSMsg(*points_msg, *laserCloudRoadResColor_pcd);
  pcl::PCDWriter pcd_writer;
std::cout << "saving...";
pcd_writer.writeBinary(all_points_filename, *laserCloudRoadResColor_pcd);
std::cout << "points saved";
}
  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;


    registration->setInputTarget(globalmap);
  }


  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new hdl_localization::PoseEstimator(
            registration,
            ros::Time::now(),
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z),
            private_nh.param<double>("cool_time_duration", 0.5))
    );
  }


  void RGBpointAssociateToMap(PointT const * const pi, pcl::PointXYZRGB * const po)
{
    double s;
    float transformAftMapped[6] = {0};
    
    s = pi->intensity - int(pi->intensity);

    // float rx = (1-s)*transformLastMapped[0] + s * transformAftMapped[0];
    // float ry = (1-s)*transformLastMapped[1] + s * transformAftMapped[1];
    // float rz = (1-s)*transformLastMapped[2] + s * transformAftMapped[2];
    // float tx = (1-s)*transformLastMapped[3] + s * transformAftMapped[3];
    // float ty = (1-s)*transformLastMapped[4] + s * transformAftMapped[4];
    // float tz = (1-s)*transformLastMapped[5] + s * transformAftMapped[5];
    float rx = transformAftMapped[0];
    float ry = transformAftMapped[1];
    float rz = transformAftMapped[2];
    float tx = transformAftMapped[3];
    float ty = transformAftMapped[4];
    float tz = transformAftMapped[5];
    //rot z（transformTobeMapped[2]）
    float x1 = cos(rz) * pi->x
            - sin(rz) * pi->y;
    float y1 = sin(rz) * pi->x
            + cos(rz) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(ry) * x2 + sin(ry) * z2 + tx;
    po->y = y2 + ty;
    po->z = -sin(ry) * x2 + cos(ry) * z2 + tz;
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    std::cout<<"DEBUG reflection_map "<<reflection_map<<std::endl;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", odom_child_frame_id);
    pose_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub.publish(odom);
  }

  /**
   * @brief convert a Eigen::Matrix to TransformedStamped
   * @param stamp           timestamp
   * @param pose            pose matrix
   * @param frame_id        frame_id
   * @param child_frame_id  child_frame_id
   * @return transform
   */
  geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
    Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
    quat.normalize();
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = quat.w();
    odom_quat.x = quat.x();
    odom_quat.y = quat.y();
    odom_quat.z = quat.z();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = pose(0, 3);
    odom_trans.transform.translation.y = pose(1, 3);
    odom_trans.transform.translation.z = pose(2, 3);
    odom_trans.transform.rotation = odom_quat;

    return odom_trans;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  
  std::string odom_child_frame_id;

  bool use_imu;
  bool invert_imu;
  ros::Subscriber imu_sub;
  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber livox_sub;
  ros::Subscriber painted_cloud_sub;
  ros::Subscriber initialpose_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher filter_pub;
  ros::Publisher map_pub;

  tf::TransformBroadcaster pose_broadcaster;
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::ImuConstPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::PointCloud<PointT>::Ptr velodynemap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // processing time buffer
  boost::circular_buffer<double> processing_time;

  Eigen::Matrix4f transform_to_map ;
};


}

PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
