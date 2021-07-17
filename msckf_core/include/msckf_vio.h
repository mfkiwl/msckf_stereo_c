/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_H
#define MSCKF_VIO_H

#include <map>
#include <set>
#include <vector>
#include <string>
#include <fstream>

//#include <Eigen/Dense>
//#include <Eigen/Geometry>

#include "common/data_msg.h"
#include "common/config_io.h"
#include "common/imu_state.h"
#include "common/cam_state.h"
#include "feature.hpp"

namespace cg {
/*
 * @brief MsckfVio Implements the algorithm in
 *    Anatasios I. Mourikis, and Stergios I. Roumeliotis,
 *    "A Multi-State Constraint Kalman Filter for Vision-aided
 *    Inertial Navigation",
 *    http://www.ee.ucr.edu/~mourikis/tech_reports/TR_MSCKF.pdf
 */
class MsckfVio {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    MsckfVio(YAML::Node cfg_cam_imu);
    // Disable copy and assign constructor
    MsckfVio(const MsckfVio&) = delete;
    MsckfVio operator=(const MsckfVio&) = delete;

    // Destructor
    ~MsckfVio() {
        pose_outfile_.close();
        debug_.close();
    }

    /*
     * @brief initialize Initialize the VIO.
     */
    bool initialize();

    /*
     * @biref resetCallback
     *    Callback function for the reset service.
     *    Note that this is NOT anytime-reset. This function should
     *    only be called before the sensor suite starts moving.
     *    e.g. while the robot is still on the ground.
     */
    bool resetCallback();

    /*
     * @brief imuCallback
     *    Callback function for the imu message.
     * @param msg IMU msg.
     */
    void imuCallback(const cg::ImuConstPtr &msg);

    /*
     * @brief featureCallback
     *    Callback function for feature measurements.
     * @param msg Stereo feature measurements.
     */
    void featureCallback(const CameraMeasurementConstPtr& msg);

    std::vector<cg::Vector3> get_path() {return path_;}

    std::vector<cg::Point3f> get_points3d() {return points3d_;}

    typedef std::shared_ptr<MsckfVio> Ptr;
    typedef std::shared_ptr<const MsckfVio> ConstPtr;

  private:
    /*
     * @brief StateServer Store one IMU states and several
     *    camera states for constructing measurement
     *    model.
     */
    struct StateServer {
      IMUState imu_state;
      CamStateServer cam_states;

      // State covariance matrix
      cg::Matrix state_cov;
      cg::Matrix continuous_noise_cov;
      StateServer() : continuous_noise_cov(cg::Matrix(12,12)) {}
    };

    /*
     * @brief loadParameters
     *    Load parameters from the parameter server.
     */
    bool loadParameters();

    /*
     * @brief publish Publish the results of VIO.
     * @param time The time stamp of output msgs.
     */
    void publish(double time_stamp);

    /*
     * @brief initializegravityAndBias
     *    Initialize the IMU bias and initial orientation
     *    based on the first few IMU readings.
     */
    void initializeGravityAndBias();

    // Filter related functions
    // Propogate the state
    void batchImuProcessing(const double& time_bound);
    void processModel(const double& time, const cg::Vector3& m_gyro, const cg::Vector3& m_acc);
    void predictNewState(const double& dt, const cg::Vector3& gyro, const cg::Vector3& acc);

    // Measurement update
    void stateAugmentation(const double& time);
    void addFeatureObservations(const CameraMeasurementConstPtr& msg);
    // This function is used to compute the measurement Jacobian
    // for a single feature observed at a single camera frame.
    // H_x 4x6, H_f 4x3
    void measurementJacobian(const StateIDType& cam_state_id, const FeatureIDType& feature_id, cg::Matrix& H_x, cg::Matrix& H_f, cg::Vector4& r);
    // This function computes the Jacobian of all measurements viewed
    // in the given camera states of this feature.
    void featureJacobian(const FeatureIDType& feature_id, const std::vector<StateIDType>& cam_state_ids, cg::Matrix &H_x, cg::VectorX &r);
    void measurementUpdate(const cg::Matrix &H, const cg::VectorX &r);
    bool gatingTest(const cg::Matrix& H, const cg::VectorX&r, const int& dof);
    void removeLostFeatures();
    void findRedundantCamStates(std::vector<StateIDType>& rm_cam_state_ids);
    void pruneCamStateBuffer();
    // Reset the system online if the uncertainty is too large.
    void onlineReset();

    YAML::Node cfg_cam_imu_;

    std::vector<cg::Vector3> path_;
    std::vector<cg::Point3f> points3d_;

    // Chi squared test table.
    static std::map<int, double> chi_squared_test_table;

    // State vector
    StateServer state_server;
    // Maximum number of camera states
    int max_cam_state_size;

    // Features used
    MapServer map_server;

    // IMU data buffer
    // This is buffer is used to handle the unsynchronization or
    // transfer delay between IMU and Image messages.
    std::vector<cg::Imu> imu_msg_buffer;

    // Indicate if the gravity vector is set.
    bool is_gravity_set;

    // Indicate if the received image is the first one. The
    // system will start after receiving the first image.
    bool is_first_img;

    // The position uncertainty threshold is used to determine
    // when to reset the system online. Otherwise, the ever-
    // increaseing uncertainty will make the estimation unstable.
    // Note this online reset will be some dead-reckoning.
    // Set this threshold to nonpositive to disable online reset.
    double position_std_threshold;

    // Tracking rate
    double tracking_rate;

    // Threshold for determine keyframes
    double translation_threshold;
    double rotation_threshold;
    double tracking_rate_threshold;

    // Framte rate of the stereo images. This variable is
    // only used to determine the timing threshold of
    // each iteration of the filter.
    double frame_rate;

    std::ofstream pose_outfile_;
    std::ofstream debug_;

    int n_pub = 0;
};

typedef MsckfVio::Ptr MsckfVioPtr;
typedef MsckfVio::ConstPtr MsckfVioConstPtr;

} // namespace msckf_vio

#endif
