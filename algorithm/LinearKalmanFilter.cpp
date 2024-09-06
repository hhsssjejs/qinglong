//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "LinearKalmanFilter.h"


KalmanFilterEstimate::KalmanFilterEstimate()
{
  xHat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(6, 6, 6, 6) = Eigen::Matrix<scalar_t, 6, 6>::Identity();
  b_.setZero();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();  // additional
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<scalar_t, 3, 3>::Identity(), Eigen::Matrix<scalar_t, 3, 3>::Zero();
  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<scalar_t, 3, 3>::Zero(), Eigen::Matrix<scalar_t, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(0, 6, 6, 6) = -Eigen::Matrix<scalar_t, 12, 12>::Identity();
  c_.block(6, 0, 3, 6) = c2;
  c_.block(9, 0, 3, 6) = c2;
  c_(13, 11) = 1.0;
  c_(12, 8) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(6, 6, 6, 6) = dt * Eigen::Matrix<scalar_t, 6, 6>::Identity();    

  r_.setIdentity();
  feetHeights_.setZero(numContacts); 
}

void KalmanFilterEstimate::update(
    const Eigen::Matrix3d &base_rot, const Eigen::Vector3d &lin_acc,
    const std::vector<bool> &stance_phase,
    const std::vector<Eigen::Vector3d> &end_pos_world,
    const std::vector<Eigen::Vector3d> &end_vel_world) {

  // const auto& model = pinocchioInterface_.getModel();
  // auto& data = pinocchioInterface_.getData();

  // vector_t qPino(generalizedCoordinatesNum);
  // vector_t vPino(generalizedCoordinatesNum);
  // qPino.setZero();
  // qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  // qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  // vPino.setZero();
  // vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
  //     qPino.segment<3>(3),
  //     rbdState_.segment<3>(generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  // vPino.tail(actuatedDofNum) = rbdState_.segment(6 + generalizedCoordinatesNum, actuatedDofNum);

  // pinocchio::forwardKinematics(model, data, qPino, vPino);
  // pinocchio::updateFramePlacements(model, data);

  // const auto eePos = eeKinematics_->getPosition(vector_t());
  // const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

  // the covariance of the process noise
  Eigen::Matrix<scalar_t, 12, 12> q = Eigen::Matrix<scalar_t, 12, 12>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
  q.block(6, 6, 6, 6) = q_.block(6, 6, 6, 6) * footProcessNoisePosition_;

  // the covariance of the observation noise
  Eigen::Matrix<scalar_t, 14, 14> r = Eigen::Matrix<scalar_t, 14, 14>::Identity();
  r.block(0, 0, 6, 6) = r_.block(0, 0, 6, 6) * footSensorNoisePosition_;
  r.block(6, 6, 6, 6) = r_.block(6, 6, 6, 6) * footSensorNoiseVelocity_;
  const int fn = numContacts;
  r.block(12, 12, fn, fn) = r_.block(12, 12, fn, fn) * footHeightSensorNoise_;

  for (int i = 0; i < numContacts; i++)
  {
    int i1 = 3 * i;

    int qIndex = 6 + i1;
    int rIndex1 = i1;
    int rIndex2 = 6 + i1;
    int rIndex3 = 12 + i;
    bool isContact = stance_phase[i];

    scalar_t high_suspect_number(100);
    q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
    r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
    r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
    r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

    ps_.segment(3 * i, 3) = -end_pos_world[i];
    ps_.segment(3 * i, 3)[2] += footRadius_;
    vs_.segment(3 * i, 3) = -end_vel_world[i];
  }

  vector3_t g(0, 0, -9.81);
  vector3_t accel = base_rot * lin_acc + g;

  // observation (or measurement)
  Eigen::Matrix<scalar_t, 14, 1> y;
  y << ps_, vs_, feetHeights_;
  xHat_ = a_ * xHat_ + b_ * accel;  
  Eigen::Matrix<scalar_t, 12, 12> at = a_.transpose();
  Eigen::Matrix<scalar_t, 12, 12> pm = a_ * p_ * at + q;  
  Eigen::Matrix<scalar_t, 12, 14> cT = c_.transpose();
  Eigen::Matrix<scalar_t, 14, 1> yModel = c_ * xHat_;
  Eigen::Matrix<scalar_t, 14, 1> ey = y - yModel;        
  Eigen::Matrix<scalar_t, 14, 14> s = c_ * pm * cT + r;  

  Eigen::Matrix<scalar_t, 14, 1> sEy = s.lu().solve(ey);  
  xHat_ += pm * cT * sEy;                                 

  Eigen::Matrix<scalar_t, 14, 12> sC = s.lu().solve(c_);
  p_ = (Eigen::Matrix<scalar_t, 12, 12>::Identity() - pm * cT * sC) *
       pm;  

  Eigen::Matrix<scalar_t, 12, 12> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001)
  {
    p_.block(0, 2, 2, 10).setZero();
    p_.block(2, 0, 10, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }
}


void KalmanFilterEstimate::resetEstimate()
{
  xHat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(6, 6, 6, 6) = Eigen::Matrix<scalar_t, 6, 6>::Identity();
  b_.setZero();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();  // additional
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<scalar_t, 3, 3>::Identity(), Eigen::Matrix<scalar_t, 3, 3>::Zero();
  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<scalar_t, 3, 3>::Zero(), Eigen::Matrix<scalar_t, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(0, 6, 6, 6) = -Eigen::Matrix<scalar_t, 12, 12>::Identity();
  c_.block(6, 0, 3, 6) = c2;
  c_.block(9, 0, 3, 6) = c2;
  c_(13, 11) = 1.0;
  c_(12, 8) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(6, 6, 6, 6) = dt * Eigen::Matrix<scalar_t, 6, 6>::Identity();    

  r_.setIdentity();
  feetHeights_.setZero(numContacts); 
}

// void KalmanFilterEstimate::loadSettings(const std::string& taskFile, bool verbose)
// {
//   boost::property_tree::ptree pt;
//   boost::property_tree::read_info(taskFile, pt);
//   std::string prefix = "kalmanFilter.";
//   if (verbose)
//   {
//     std::cerr << "\n #### Kalman Filter Noise:";
//     std::cerr << "\n #### =============================================================================\n";
//   }

//   loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
//   loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
//   loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
//   loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
//   loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
//   loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
//   loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
// }
