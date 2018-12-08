/*
 * time_sync.cpp
 *
 *  Created on: Sep 21, 2016
 *      Authors: Michael Burri, Helen Oleynikova
 */

#include "realsense_camera/time_sync.h"

namespace realsense_camera {

TimeSyncFilter::TimeSyncFilter() : is_initialized_(false) {
}

double TimeSyncFilter::getLocalTimestamp(double device_time) {
  if (!is_initialized_) {
    std::cout << "[WARN]: Timesync filter not initialized yet! Hack "
                 "initializing now." << std::endl;
    double local_time = ros::Time::now().toSec();
    initialize(device_time, local_time);
  }
  double dt = device_time - last_update_device_time_;
  return device_time + x_(0) + dt * x_(1);
}

void TimeSyncFilter::print() const {
  std::cout << "offset: " << x_(0) << "skew: " << x_(1) << "dt" << dt_
            << std::endl;
  // std::cout << P_<< std::endl;
}

void TimeSyncFilter::reset() { is_initialized_ = 0; }

void TimeSyncFilter::updateFilter(double device_time, double local_time) {
  if (!is_initialized_) {
    initialize(device_time, local_time);
    return;
  }

  double dt = device_time - last_update_device_time_;

  if (dt < kUpdateRate) return;

  dt_ = dt;
  Eigen::Matrix2d F;
  F << 1, dt_, 0, 1;

  // Prediction
  Eigen::Vector2d x_pred = F * x_;

  // check for outlier
  double measurement_residual = local_time - device_time - H_ * x_pred;

  if (measurement_residual > 2 * kSigmaMeasurementTimeOffset) {
    // std::cout << "Timesync outlier" << std::endl;
    return;
  }

  P_ = F * P_ * F.transpose() + dt_ * Q_;

  // Update
  double S = H_ * P_ * H_.transpose() + R_;

  Eigen::Vector2d K;
  K = P_ * H_.transpose() * (1 / S);

  x_ = x_pred + K * measurement_residual;
  P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;

  last_update_device_time_ = device_time;

  // print();
}

void TimeSyncFilter::initialize(double device_time, double local_time) {
  x_.setZero();
  x_[0] = local_time - device_time;

  P_.setZero();
  P_(0, 0) = kSigmaInitOffset * kSigmaInitOffset;
  P_(1, 1) = kSigmaInitSkew * kSigmaInitSkew;

  Q_.setZero();
  Q_(0, 0) = kSigmaOffset * kSigmaOffset;
  Q_(1, 1) = kSigmaSkew * kSigmaSkew;

  R_ = kSigmaMeasurementTimeOffset * kSigmaMeasurementTimeOffset;

  H_.setZero();
  H_(0, 0) = 1;

  last_update_device_time_ = device_time;
  is_initialized_ = true;
}

bool TimeSyncFilter::isInitialized() const {
  return is_initialized_;
}

}  // namespace realsense_camera
