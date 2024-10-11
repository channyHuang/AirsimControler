#include <iostream>

#include "estimator/ukf.hpp"
#include "sensor/imu.hpp"

#include "messageStruct.h"
#include <thread>

namespace cg {

ANGULAR_ERROR State::kAngError = ANGULAR_ERROR::LOCAL_ANGULAR_ERROR;

class UKFFusionNode {
 public:
  UKFFusionNode() {
    double acc_n, gyr_n, acc_w, gyr_w, sigma_pv, sigma_rp, sigma_yaw;
    // nh.param("acc_noise", acc_n, 1e-2);
    // nh.param("gyr_noise", gyr_n, 1e-4);
    // nh.param("acc_bias_noise", acc_w, 1e-6);
    // nh.param("gyr_bias_noise", gyr_w, 1e-8);

    // nh.param("init_sigma_pv", sigma_pv, 0.01);
    // nh.param("init_sigma_rp", sigma_rp, 0.01);
    // nh.param("init_sigma_yaw", sigma_yaw, 5.0);

    sigma_rp *= kDegreeToRadian;
    sigma_yaw *= kDegreeToRadian;

    ukf_ptr_ = std::make_unique<UKF>(sigma_pv, sigma_pv, sigma_rp, sigma_yaw, acc_w, gyr_w);
    ukf_ptr_->imu_model_ = std::make_shared<IMU>(ukf_ptr_->state_ptr_, acc_n, gyr_n, acc_w, gyr_w);
    ukf_ptr_->predictor_ptr_ = std::dynamic_pointer_cast<IMU>(ukf_ptr_->imu_model_);
    // ukf_ptr_->observer_ptr_ = std::make_shared<Odom6Dof>();

    // imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic_imu, 10, boost::bind(&UKFFusionNode::imu_callback, this, _1));
    // vo_sub_ = nh.subscribe(topic_vo, 10, &UKFFusionNode::vo_callback, this);

    // Tcb = Utils::getTransformEigen(pnh, "cam0/T_cam_imu");
  }

  ~UKFFusionNode() {}

  void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    Eigen::Vector3d acc, gyr;
    acc[0] = imu_msg->linear_acceleration.x();
    acc[1] = imu_msg->linear_acceleration.y();
    acc[2] = imu_msg->linear_acceleration.z();
    gyr[0] = imu_msg->angular_velocity.x();
    gyr[1] = imu_msg->angular_velocity.y();
    gyr[2] = imu_msg->angular_velocity.z();

    ukf_ptr_->predict(std::make_shared<ImuData>(imu_msg->header.stamp.toSec(), acc, gyr));

    if (!ukf_ptr_->predictor_ptr_->inited_) {
      if(!ukf_ptr_->predictor_ptr_->init(imu_msg->header.stamp.toSec())) {
        return;
      }
    }
    
  }

 private:
  Eigen::Isometry3d Tcb;
  Eigen::Isometry3d Tvw;

  UKFPtr ukf_ptr_;
};
}
