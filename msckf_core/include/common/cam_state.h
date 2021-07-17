/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_CAM_STATE_H
#define MSCKF_VIO_CAM_STATE_H

#include <map>
#include <vector>
//#include <Eigen/Dense>

#include "imu_state.h"

#include "maths/vector.h"
#include "kinematics/quarternion.h"
#include "kinematics/transform.h"

namespace cg {
/*
 * @brief CAMState Stored camera state in order to form measurement model.
 */
struct CAMState {
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // An unique identifier for the CAM state.
  StateIDType id;

  // Time when the state is recorded
  double time;

  // Orientation
  // Take a vector from the world frame to the camera frame.
  cg::Quarternion orientation;

  // Position of the camera frame in the world frame.
  cg::Vector3 position;

  // These two variables should have the same physical
  // interpretation with `orientation` and `position`.
  // There two variables are used to modify the measurement
  // Jacobian matrices to make the observability matrix
  // have proper null space.
  cg::Quarternion orientation_null;
  cg::Vector3 position_null;

  // Takes a vector from the cam0 frame to the cam1 frame.
  static cg::EuclideanTransform T_cam0_cam1;

  CAMState(): id(0), time(0) {}

  CAMState(const StateIDType& new_id ): id(new_id), time(0) {}
};

//typedef std::map<StateIDType, CAMState, std::less<int>,
//        Eigen::aligned_allocator<std::pair<const StateIDType, CAMState> > > CamStateServer;

    typedef std::map<StateIDType, CAMState, std::less<int> > CamStateServer;

} // namespace msckf_vio

#endif // MSCKF_VIO_CAM_STATE_H
