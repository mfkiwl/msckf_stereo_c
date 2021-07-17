/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_IMU_STATE_H
#define MSCKF_VIO_IMU_STATE_H

#include <map>
#include <vector>
//#include <Eigen/Dense>
//#include <Eigen/Geometry>

#include "maths/vector.h"
#include "kinematics/quarternion.h"
#include "kinematics/rotation_matrix.h"
#include "kinematics/transform.h"

#define GRAVITY_ACCELERATION 9.81

namespace cg {

/*
 * @brief IMUState State for IMU
 */
struct IMUState {
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef long long int StateIDType;

  // An unique identifier for the IMU state.
  StateIDType id;

  // id for next IMU state
  static StateIDType next_id;

  // Time when the state is recorded
  double time;

  // Orientation
  // Take a vector from the world frame to the IMU (body) frame.
  cg::Quarternion orientation;

  // Position of the IMU (body) frame in the world frame.
  cg::Vector3 position;

  // Velocity of the IMU (body) frame in the world frame.
  cg::Vector3 velocity;

  // Bias for measured angular velocity and acceleration.
  cg::Vector3 gyro_bias;
  cg::Vector3 acc_bias;

  // Transformation between the IMU and the left camera (cam0)
  cg::RotationMatrix R_imu_cam0;
  cg::Vector3 t_cam0_imu;

  // These three variables should have the same physical
  // interpretation with `orientation`, `position`, and
  // `velocity`. There three variables are used to modify
  // the transition matrices to make the observability matrix
  // have proper null space.
  cg::Quarternion orientation_null;
  cg::Vector3 position_null;
  cg::Vector3 velocity_null;

  // Process noise
  static double gyro_noise;
  static double acc_noise;
  static double gyro_bias_noise;
  static double acc_bias_noise;

  // Gravity vector in the world frame
  static cg::Vector3 gravity;

  // Transformation offset from the IMU frame to
  // the body frame. The transformation takes a
  // vector from the IMU frame to the body frame.
  // The z axis of the body frame should point upwards.
  // Normally, this transform should be identity.
  static cg::EuclideanTransform T_imu_body;

  IMUState(): id(0), time(0) {}

  IMUState(const StateIDType& new_id): id(new_id), time(0) {}

};

typedef IMUState::StateIDType StateIDType;

} // namespace msckf_vio

#endif // MSCKF_VIO_IMU_STATE_H
