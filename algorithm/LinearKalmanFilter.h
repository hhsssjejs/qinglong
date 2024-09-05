//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <Typesss.h>

class KalmanFilterEstimate
{
public:
  KalmanFilterEstimate();

  void update(const Eigen::Matrix3d &base_rot, const Eigen::Vector3d &lin_acc,
              const std::vector<bool> &stance_phase,
              const std::vector<Eigen::Vector3d> &end_pos_world,
              const std::vector<Eigen::Vector3d> &end_vel_world);

  void loadSettings(const std::string& taskFile, bool verbose);

  vector_t getBodyVelWorld()
  {
    vector_t body_vel(6);
    body_vel.head(3) = xHat_.segment<3>(0);
    body_vel.tail(3) = xHat_.segment<3>(3);
    return std::move(body_vel);
  }
  scalar_t dt = 0.001;

protected:

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

private:
  Eigen::Matrix<scalar_t, 12, 1> xHat_;
  Eigen::Matrix<scalar_t, 6, 1> ps_;
  Eigen::Matrix<scalar_t, 6, 1> vs_;
  Eigen::Matrix<scalar_t, 12, 12> a_;
  Eigen::Matrix<scalar_t, 12, 12> q_;
  Eigen::Matrix<scalar_t, 12, 12> p_;
  Eigen::Matrix<scalar_t, 14, 14> r_;
  Eigen::Matrix<scalar_t, 12, 3> b_;
  Eigen::Matrix<scalar_t, 14, 12> c_;

};
