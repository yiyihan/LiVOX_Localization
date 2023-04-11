#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pclomp/ndt_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <hdl_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;
  using PointType = pcl::PointXYZL;
  /**
   * @brief constructor
   * @param registration        registration method
   * @param stamp               timestamp
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const ros::Time& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double cool_time_duration = 1.0)
    : init_stamp(stamp),
      registration(registration),
      cool_time_duration(cool_time_duration)
  {
    process_noise = Eigen::MatrixXf::Identity(16, 16);
    process_noise.middleRows(0, 3) *= 1.0;
    process_noise.middleRows(3, 3) *= 1.0;
    process_noise.middleRows(6, 4) *= 0.5;
    process_noise.middleRows(10, 3) *= 1e-6;
    process_noise.middleRows(13, 3) *= 1e-6;

    Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
    measurement_noise.middleRows(0, 3) *= 0.01;
    measurement_noise.middleRows(3, 4) *= 0.001;

    Eigen::VectorXf mean(16);
    mean.middleRows(0, 3) = pos;
    mean.middleRows(3, 3).setZero();
    mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());
    mean.middleRows(10, 3).setZero();
    mean.middleRows(13, 3).setZero();

    Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

    PoseSystem system;
    ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
  }

  /**
   * @brief predict
   * @param stamp    timestamp
   * @param acc      acceleration
   * @param gyro     angular velocity
   */
  void predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro) {
    if((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
      prev_stamp = stamp;
      return;
    }

    double dt = (stamp - prev_stamp).toSec();
    prev_stamp = stamp;

    ukf->setProcessNoiseCov(process_noise * dt);
    ukf->system.dt = dt;

    Eigen::VectorXf control(6);
    control.head<3>() = acc;
    control.tail<3>() = gyro;

    ukf->predict(control);
  }

  /**
   * @brief correct
   * @param cloud   input cloud
   * @return cloud aligned to the globalmap
   */
  pcl::PointCloud<PointT>::Ptr correct(const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

    init_guess.block<3, 3>(0, 0) = quat().toRotationMatrix();
    init_guess.block<3, 1>(0, 3) = pos();
    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->setInputSource(cloud);
    registration->align(*aligned, init_guess);

    // pcl::transformPointCloud (*aligned, *aligned_livox, transform_1.inverse());

    Eigen::Matrix4f trans = registration->getFinalTransformation();
    // std::cout<<"trans is:"<<trans<<std::endl;
    Eigen::Vector3f p = trans.block<3, 1>(0, 3);
    Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

    if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
      q.coeffs() *= -1.0f;
    }

    Eigen::VectorXf observation(7);
    observation.middleRows(0, 3) = p;
    observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());

    ukf->correct(observation);
    // return aligned_livox;
    return aligned;
  }

  pcl::PointCloud<PointType>::Ptr correct_with_label(const pcl::PointCloud<PointType>::ConstPtr& cloud) {
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // transform_1(0,0) = 9.99997456e-01 ;
    // transform_1(0,1) = -2.18275887e-03  ;
    // transform_1(0,2) = 3.93688199e-04 ;
    // transform_1(0,3) = -6.25288242e-02 ;
    // transform_1(1,0) = 2.18272349e-03  ;
    // transform_1(1,1) = 9.99997762e-01 ;
    // transform_1(1,2) =  3.55326510e-05 ;
    // transform_1(1,3) = -2.44046831e-02  ;
    // transform_1(2,0) = -3.93765045e-04 ;
    // transform_1(2,1) = -3.46785103e-05  ;
    // transform_1(2,2) = 9.99999878e-01  ;
    // transform_1(2,3) = 1.65635988e-01 ;
    init_guess.block<3, 3>(0, 0) = quat().toRotationMatrix();
    init_guess.block<3, 1>(0, 3) = pos();
    pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
    // pcl::PointCloud<PointType>::Ptr aligned_livox(new pcl::PointCloud<PointType>());
    Registration->setInputSource(cloud);
    Registration->align(*aligned, init_guess);

    // pcl::transformPointCloud (*aligned, *aligned_livox, transform_1.inverse());

    Eigen::Matrix4f trans = Registration->getFinalTransformation();
    // std::cout<<"trans is:"<<trans<<std::endl;
    Eigen::Vector3f p = trans.block<3, 1>(0, 3);
    Eigen::Quaternionf q(trans.block<3, 3>(0, 0));

    if(quat().coeffs().dot(q.coeffs()) < 0.0f) {
      q.coeffs() *= -1.0f;
    }

    Eigen::VectorXf observation(7);
    observation.middleRows(0, 3) = p;
    observation.middleRows(3, 4) = Eigen::Vector4f(q.w(), q.x(), q.y(), q.z());

    ukf->correct(observation);
    // return aligned_livox;
    return aligned;
  }


  /* getters */
  Eigen::Vector3f pos() const {
    return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
  }

  Eigen::Vector3f vel() const {
    return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
  }

  Eigen::Quaternionf quat() const {
    return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
  }

  Eigen::Matrix4f matrix() const {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1(0,0) = 9.98329310e-01 ;
    transform_1(0,1) = -5.71876411e-02  ;
    transform_1(0,2) = -8.25610350e-03 ;
    transform_1(0,3) = 7.67874558e+00 ;
    transform_1(1,0) =  5.72705895e-02  ;
    transform_1(1,1) = 9.98306723e-01 ;
    transform_1(1,2) =  1.01865823e-02 ;
    transform_1(1,3) = 1.30356913e+00  ;
    transform_1(2,0) = 7.65957701e-03 ;
    transform_1(2,1) = -1.06423956e-02  ;
    transform_1(2,2) = 9.99914031e-01  ;
    transform_1(2,3) = -9.03116121e-02 ;

    m.block<3, 3>(0, 0) = quat().toRotationMatrix();
    m.block<3, 1>(0, 3) = pos();
    // return transform_1 * m;
    return m;
  }

private:
  ros::Time init_stamp;         // when the estimator was initialized
  ros::Time prev_stamp;         // when the estimator was updated last time
  double cool_time_duration;    //

  Eigen::MatrixXf process_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;

  pcl::Registration<PointT, PointT>::Ptr registration;
  pcl::Registration<PointType, PointType>::Ptr Registration;

};

}

#endif // POSE_ESTIMATOR_HPP
