/*
 * time_sync.h
 *
 *  Created on: Sep 21, 2016
 *      Authors: Michael Burri, Helen Oleynikova
 */

#include <iostream>

#include <Eigen/Dense>
#include <ros/ros.h>

#ifndef REALSENSE_CAMERA_TIME_SYNC_H_
#define REALSENSE_CAMERA_TIME_SYNC_H_

namespace realsense_camera {

class TimeSyncFilter {
 public:
  static constexpr double kSigmaInitOffset = 2e-3;
  static constexpr double kSigmaInitSkew = 1e-3;
  static constexpr double kSigmaMeasurementTimeOffset = 2e-3;
  static constexpr double kSigmaSkew = 2e-6;
  static constexpr double kSigmaOffset = 1e-5;
  static constexpr double kUpdateRate = 0.1; // (1/sec)

  TimeSyncFilter();
  ~TimeSyncFilter() {};
  double getLocalTimestamp(double device_time);
  void updateFilter(double device_time, double local_time);
  void print() const;
  void reset();
  bool isInitialized() const;
 private:
  void initialize(double device_time, double local_time);
  bool isOutlier(double device_time, double local_time);

  Eigen::Vector2d x_;
  Eigen::Matrix2d P_;
  Eigen::Matrix2d Q_;
  double R_;
  Eigen::Matrix<double, 1, 2> H_;
  bool is_initialized_;
  double last_update_device_time_;
  double dt_;
};

}  // namespace realsense_camera

#endif  // REALSENSE_CAMERA_TIME_SYNC_H_
