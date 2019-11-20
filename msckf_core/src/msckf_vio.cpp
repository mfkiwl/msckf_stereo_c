/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include "msckf_vio.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <chrono>

#include <boost/math/distributions/chi_squared.hpp>

#include <Eigen/Dense>
//#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>

//#include <opencv2/core/core.hpp>

#include "kinematics/convertor.h"
#include "maths/svd_fulluv.h"

namespace mynt {
    // Static member variables in IMUState class.
    StateIDType IMUState::next_id = 0;
    double IMUState::gyro_noise = 0.001;
    double IMUState::acc_noise = 0.01;
    double IMUState::gyro_bias_noise = 0.001;
    double IMUState::acc_bias_noise = 0.01;
    mynt::Vector3 IMUState::gravity = mynt::Vector3({0, 0, -GRAVITY_ACCELERATION});
    mynt::EuclideanTransform IMUState::T_imu_body = mynt::EuclideanTransform();

    // Static member variables in CAMState class.
    mynt::EuclideanTransform CAMState::T_cam0_cam1 = mynt::EuclideanTransform();

    // Static member variables in Feature class.
    FeatureIDType Feature::next_id = 0;
    double Feature::observation_noise = 0.01;
    Feature::OptimizationConfig Feature::optimization_config;

    std::map<int, double> MsckfVio::chi_squared_test_table;

    MsckfVio::MsckfVio(YAML::Node cfg_cam_imu) :
        cfg_cam_imu_(cfg_cam_imu),
        is_gravity_set(false),
        is_first_img(true) {
        return;
    }

    bool MsckfVio::loadParameters() {
        YAML::Node cfg_msckfvio = YAML::LoadFile("../config/app_msckfvio.yaml");
        frame_rate = cfg_msckfvio["frame_rate"].as<double>();
        position_std_threshold = cfg_msckfvio["position_std_threshold"].as<double>();
        rotation_threshold = cfg_msckfvio["rotation_threshold"].as<double>();
        translation_threshold = cfg_msckfvio["translation_threshold"].as<double>();
        tracking_rate_threshold = cfg_msckfvio["tracking_rate_threshold"].as<double>();

        // Feature optimization parameters
        Feature::optimization_config.translation_threshold = cfg_msckfvio["feature/config/translation_threshold"].as<double>();

        // Noise related parameters
        IMUState::gyro_noise = cfg_msckfvio["noise/gyro"].as<double>();
        IMUState::acc_noise = cfg_msckfvio["noise/acc"].as<double>();
        IMUState::gyro_bias_noise = cfg_msckfvio["noise/gyro_bias"].as<double>();
        IMUState::acc_bias_noise = cfg_msckfvio["noise/acc_bias"].as<double>();
        Feature::observation_noise = cfg_msckfvio["noise/feature"].as<double>();

        // Use variance instead of standard deviation.
        IMUState::gyro_noise *= IMUState::gyro_noise;
        IMUState::acc_noise *= IMUState::acc_noise;
        IMUState::gyro_bias_noise *= IMUState::gyro_bias_noise;
        IMUState::acc_bias_noise *= IMUState::acc_bias_noise;
        Feature::observation_noise *= Feature::observation_noise;

        // Set the initial IMU state.
        // The intial orientation and position will be set to the origin
        // implicitly. But the initial velocity and bias can be
        // set by parameters.
        // TODO: is it reasonable to set the initial bias to 0?
        state_server.imu_state.velocity = cfg_msckfvio["initial_state/velocity"].as<mynt::Vector3>();

        // The initial covariance of orientation and position can be
        // set to 0. But for velocity, bias and extrinsic parameters,
        // there should be nontrivial uncertainty.
        double gyro_bias_cov, acc_bias_cov, velocity_cov;
        velocity_cov  = cfg_msckfvio["initial_covariance/velocity"].as<double>();
        gyro_bias_cov = cfg_msckfvio["initial_covariance/gyro_bias"].as<double>();
        acc_bias_cov  = cfg_msckfvio["initial_covariance/acc_bias"].as<double>();

        double extrinsic_rotation_cov, extrinsic_translation_cov;
        extrinsic_rotation_cov = cfg_msckfvio["initial_covariance/extrinsic_rotation_cov"].as<double>();
        extrinsic_translation_cov = cfg_msckfvio["initial_covariance/extrinsic_translation_cov"].as<double>();

        state_server.state_cov = mynt::Matrix(21, 21);
        for (int i = 3; i < 6; ++i)
            state_server.state_cov(i, i) = gyro_bias_cov;
        for (int i = 6; i < 9; ++i)
            state_server.state_cov(i, i) = velocity_cov;
        for (int i = 9; i < 12; ++i)
            state_server.state_cov(i, i) = acc_bias_cov;
        for (int i = 15; i < 18; ++i)
            state_server.state_cov(i, i) = extrinsic_rotation_cov;
        for (int i = 18; i < 21; ++i)
            state_server.state_cov(i, i) = extrinsic_translation_cov;

        // Transformation offsets between the frames involved.
        mynt::Matrix m4_cam0_imu = cfg_cam_imu_["cam0"]["T_cam_imu"].as<mynt::Mat4>();

        mynt::EuclideanTransform T_cam0_imu = m4_cam0_imu.inv();
        state_server.imu_state.R_imu_cam0 = T_cam0_imu.rotation_matrix().transpose();
        state_server.imu_state.t_cam0_imu = T_cam0_imu.translation();

        mynt::Matrix m4_cam1_cam0 = cfg_cam_imu_["cam1"]["T_cn_cnm1"].as<mynt::Mat4>();
        CAMState::T_cam0_cam1 = m4_cam1_cam0; // 坐标系变换

        mynt::Matrix m4_imu_body = cfg_cam_imu_["T_imu_body"].as<mynt::Mat4>();
        IMUState::T_imu_body = m4_imu_body.inv();

        // Maximum number of camera states to be stored
        max_cam_state_size = cfg_msckfvio["max_cam_state_size"].as<double>();

        printf("MsckfVio begin ===========================================\n");
        
        printf("frame rate: %f\n", frame_rate);
        printf("position std threshold: %f\n", position_std_threshold);
        printf("Keyframe rotation threshold: %f\n", rotation_threshold);
        printf("Keyframe translation threshold: %f\n", translation_threshold);
        printf("Keyframe tracking rate threshold: %f\n", tracking_rate_threshold);
        printf("gyro noise: %.10f\n", IMUState::gyro_noise);
        printf("gyro bias noise: %.10f\n", IMUState::gyro_bias_noise);
        printf("acc noise: %.10f\n", IMUState::acc_noise);
        printf("acc bias noise: %.10f\n", IMUState::acc_bias_noise);
        printf("observation noise: %.10f\n", Feature::observation_noise);
        printf("initial velocity: %f, %f, %f\n",
            state_server.imu_state.velocity[0],
            state_server.imu_state.velocity[1],
            state_server.imu_state.velocity[2]);
        printf("initial gyro bias cov: %f\n", gyro_bias_cov);
        printf("initial acc bias cov: %f\n", acc_bias_cov);
        printf("initial velocity cov: %f\n", velocity_cov);
        printf("initial extrinsic rotation cov: %f\n", extrinsic_rotation_cov);
        printf("initial extrinsic translation cov: %f\n", extrinsic_translation_cov);

        printf("max camera state #: %d\n", max_cam_state_size);

        std::cout << "state_server.imu_state.R_imu_cam0:\n" << state_server.imu_state.R_imu_cam0 << std::endl;
        std::cout << "state_server.imu_state.t_cam0_imu:\n" << state_server.imu_state.t_cam0_imu << std::endl;
        std::cout << "T_cam0_cam1:\n" << CAMState::T_cam0_cam1 << std::endl;
        std::cout << "T_imu_body:\n" << IMUState::T_imu_body << std::endl;

        printf("MsckfVio end ===========================================\n");

        return true;
    }

    bool MsckfVio::initialize() {
        if (!loadParameters())
            return false;
        std::cout << "Finish loading parameters..." << std::endl;

        // Initialize state server
        state_server.continuous_noise_cov = mynt::Matrix(12, 12);
        state_server.continuous_noise_cov.set_mat(0, 0, mynt::Matrix::eye(3) * IMUState::gyro_noise);
        state_server.continuous_noise_cov.set_mat(3, 3, mynt::Matrix::eye(3) * IMUState::gyro_bias_noise);
        state_server.continuous_noise_cov.set_mat(6, 6, mynt::Matrix::eye(3) * IMUState::acc_noise);
        state_server.continuous_noise_cov.set_mat(9, 9, mynt::Matrix::eye(3) * IMUState::acc_bias_noise);

        // Initialize the chi squared test table with confidence level 0.95.
        for (int i = 1; i < 100; ++i) {
            boost::math::chi_squared chi_squared_dist(i);
            chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
        }

        pose_outfile_.open("pose_out.txt");

        debug_.open("debug_msckfvio.txt");

        return true;
    }

    void MsckfVio::imuCallback(const mynt::ImuConstPtr &msg) {
        // IMU msgs are pushed backed into a buffer instead of
        // being processed immediately. The IMU msgs are processed
        // when the next image is available, in which way, we can
        // easily handle the transfer delay.
        imu_msg_buffer.push_back(*msg);

        if (!is_gravity_set) {
            if (imu_msg_buffer.size() < 200) {
                return;
            }
            //if (imu_msg_buffer.size() < 10) return;
            initializeGravityAndBias();
            is_gravity_set = true;
        }

        return;
    }

    void MsckfVio::initializeGravityAndBias() {
        // Initialize gravity and gyro bias.
        mynt::Vector3 sum_angular_vel;
        mynt::Vector3 sum_linear_acc;

        for (const auto &imu_msg : imu_msg_buffer) {
            mynt::Vector3 angular_vel = imu_msg.angular_velocity;
            mynt::Vector3 linear_acc  = imu_msg.linear_acceleration;

            sum_angular_vel += angular_vel;
            sum_linear_acc  += linear_acc;
        }

        state_server.imu_state.gyro_bias = sum_angular_vel / imu_msg_buffer.size();
        //IMUState::gravity =
        //  -sum_linear_acc / imu_msg_buffer.size();
        // This is the gravity in the IMU frame.
        mynt::Vector3 gravity_imu = sum_linear_acc / imu_msg_buffer.size();

        // Initialize the initial orientation, so that the estimation
        // is consistent with the inertial frame.
        double gravity_norm = gravity_imu.l2norm();
        IMUState::gravity = mynt::Vector3({0.0, 0.0, -gravity_norm});

        std::cout << "===g: " << gravity_norm << std::endl;

        // TODO: verify
        state_server.imu_state.orientation = mynt::from_two_vector(gravity_imu, -IMUState::gravity).transpose().quarternion();

        std::cout << "state_server.imu_state.orientation: " << state_server.imu_state.orientation << std::endl;

        return;
    }

    bool MsckfVio::resetCallback() {
        std::cout << "Start resetting msckf vio..." << std::endl;
        // Temporarily shutdown the subscribers to prevent the
        // state from updating.
//        feature_sub.shutdown();
//        imu_sub.shutdown();

        // Reset the IMU state.
        IMUState &imu_state = state_server.imu_state;
        imu_state.time = 0.0;
        imu_state.orientation = mynt::Quarternion();
        imu_state.position = mynt::Vector3();
        imu_state.velocity = mynt::Vector3();
        imu_state.gyro_bias = mynt::Vector3();
        imu_state.acc_bias = mynt::Vector3();
        imu_state.orientation_null = mynt::Quarternion();
        imu_state.position_null = mynt::Vector3();
        imu_state.velocity_null = mynt::Vector3();

        // Remove all existing camera states.
        state_server.cam_states.clear();

        // Reset the state covariance.
        YAML::Node cfg_msckfvio = YAML::LoadFile("../config/app_msckfvio.yaml");
        double gyro_bias_cov, acc_bias_cov, velocity_cov;
        velocity_cov  = cfg_msckfvio["initial_covariance/velocity"].as<double>();
        gyro_bias_cov = cfg_msckfvio["initial_covariance/gyro_bias"].as<double>();
        acc_bias_cov  = cfg_msckfvio["initial_covariance/acc_bias"].as<double>();
        double extrinsic_rotation_cov, extrinsic_translation_cov;
        extrinsic_rotation_cov = cfg_msckfvio["initial_covariance/extrinsic_rotation_cov"].as<double>();
        extrinsic_translation_cov = cfg_msckfvio["initial_covariance/extrinsic_translation_cov"].as<double>();

        state_server.state_cov = mynt::Matrix(21, 21);
        for (int i = 3; i < 6; ++i)
            state_server.state_cov(i, i) = gyro_bias_cov;
        for (int i = 6; i < 9; ++i)
            state_server.state_cov(i, i) = velocity_cov;
        for (int i = 9; i < 12; ++i)
            state_server.state_cov(i, i) = acc_bias_cov;
        for (int i = 15; i < 18; ++i)
            state_server.state_cov(i, i) = extrinsic_rotation_cov;
        for (int i = 18; i < 21; ++i)
            state_server.state_cov(i, i) = extrinsic_translation_cov;

        // Clear all exsiting features in the map.
        map_server.clear();

        // Clear the IMU msg buffer.
        imu_msg_buffer.clear();

        // Reset the starting flags.
        is_gravity_set = false;
        is_first_img = true;

        // Restart the subscribers.
//        imu_sub = nh.subscribe("imu", 100, &MsckfVio::imuCallback, this);
//        feature_sub = nh.subscribe("features", 40, &MsckfVio::featureCallback, this);

        // TODO: When can the reset fail?
        std::cout << "Resetting msckf vio completed..." << std::endl;
        return true;
    }

    void MsckfVio::featureCallback(const CameraMeasurementConstPtr &msg) {
        // Return if the gravity vector has not been set.
        if (!is_gravity_set)
            return;

        // Start the system if the first image is received.
        // The frame where the first image is received will be
        // the origin.
        if (is_first_img) {
            is_first_img = false;
            state_server.imu_state.time = msg->time_stamp;
        }

        std::vector<FeatureMeasurement> features = msg->features;

        static double max_processing_time = 0.0;
        static int critical_time_cntr = 0;
        auto processing_start_time = std::chrono::system_clock::now();

        // Propogate the IMU state.
        // that are received before the image msg.
        auto start_time = std::chrono::system_clock::now();
        batchImuProcessing(msg->time_stamp);
        double imu_processing_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

        // Augment the state vector.
        start_time = std::chrono::system_clock::now();
        stateAugmentation(msg->time_stamp);
        double state_augmentation_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

        // Add new observations for existing features or new
        // features in the map server.
        start_time = std::chrono::system_clock::now();
        addFeatureObservations(msg);
        double add_observations_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

        // Perform measurement update if necessary.
        start_time = std::chrono::system_clock::now();
        removeLostFeatures();
        double remove_lost_features_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

        start_time = std::chrono::system_clock::now();
        pruneCamStateBuffer();
        double prune_cam_states_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

        // Publish the odometry.
        start_time = std::chrono::system_clock::now();
        publish(msg->time_stamp);
        double publish_time = std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count();

        n_pub++;

        // Reset the system if necessary.
        onlineReset();

        auto processing_end_time = std::chrono::system_clock::now();
        double processing_time = std::chrono::duration<double>(processing_end_time - processing_start_time).count();
        if (processing_time > 1.0 / frame_rate) {
            ++critical_time_cntr;
            printf("\033[1;31mTotal processing time %f/%d...\033[0m\n", processing_time, critical_time_cntr);
            printf("IMU processing time: %f/%f\n", imu_processing_time, imu_processing_time/processing_time);
            printf("State augmentation time: %f/%f\n", state_augmentation_time, state_augmentation_time/processing_time);
            printf("Add observations time: %f/%f\n", add_observations_time, add_observations_time/processing_time);
            printf("Remove lost features time: %f/%f\n", remove_lost_features_time, remove_lost_features_time / processing_time);
            printf("Remove camera states time: %f/%f\n", prune_cam_states_time, prune_cam_states_time / processing_time);
            printf("Publish time: %f/%f\n", publish_time, publish_time/processing_time);
        }

        return;
    }

    void MsckfVio::batchImuProcessing(const double &time_bound) {
        // Counter how many IMU msgs in the buffer are used.
        int used_imu_msg_cntr = 0;

        for (const auto &imu_msg : imu_msg_buffer) {
            double imu_time = imu_msg.time_stamp;
            if (imu_time < state_server.imu_state.time) {
                ++used_imu_msg_cntr;
                continue;
            }
            if (imu_time > time_bound)
                break;

            // Convert the msgs.
            mynt::Vector3 m_gyro, m_acc;
            m_gyro = imu_msg.angular_velocity;
            m_acc  = imu_msg.linear_acceleration;

            // Execute process model.
            processModel(imu_time, m_gyro, m_acc);
            ++used_imu_msg_cntr;
        }

        // Set the state ID for the new IMU state.
        state_server.imu_state.id = IMUState::next_id++;

        // Remove all used IMU msgs.
        imu_msg_buffer.erase(imu_msg_buffer.begin(), imu_msg_buffer.begin() + used_imu_msg_cntr);

        return;
    }

    void MsckfVio::processModel(const double &time, const mynt::Vector3 &m_gyro, const mynt::Vector3 &m_acc) {
        // Remove the bias from the measured gyro and acceleration
        IMUState &imu_state = state_server.imu_state;
        mynt::Vector3 gyro = m_gyro - imu_state.gyro_bias;
        mynt::Vector3 acc  = m_acc -  imu_state.acc_bias;
        double dtime = time - imu_state.time;

        // Compute discrete transition and noise covariance matrix
        mynt::Matrix F(21, 21);
        mynt::Matrix G(21, 12);

        F.set_mat(0,  0, -mynt::skew_symmetric(gyro));
        F.set_mat(0,  3, -mynt::Matrix::eye(3));
        F.set_mat(6,  0, -imu_state.orientation.rotation_matrix().transpose() * mynt::skew_symmetric(acc));
        F.set_mat(6,  9, -imu_state.orientation.rotation_matrix().transpose());
        F.set_mat(12, 6,  mynt::Matrix::eye(3));

        G.set_mat(0, 0, -mynt::Matrix::eye(3));
        G.set_mat(3, 3,  mynt::Matrix::eye(3));
        G.set_mat(6, 6, -imu_state.orientation.rotation_matrix().transpose());
        G.set_mat(9, 9,  mynt::Matrix::eye(3));

        // Approximate matrix exponential to the 3rd order,
        // which can be considered to be accurate enough assuming dtime is within 0.01s.
        mynt::Matrix Fdt = F * dtime;
        mynt::Matrix Fdt_square = Fdt * Fdt;
        mynt::Matrix Fdt_cube = Fdt_square * Fdt;
        mynt::Matrix Phi = mynt::Matrix::eye(21) + Fdt + 0.5 * Fdt_square + (1.0 / 6.0) * Fdt_cube;

        // Propogate the state using 4th order Runge-Kutta
        predictNewState(dtime, gyro, acc);

        // Modify the transition matrix
        mynt::RotationMatrix R_kk_1 = imu_state.orientation_null.rotation_matrix();
        Phi.set_mat(0, 0, imu_state.orientation.rotation_matrix() * R_kk_1.transpose());

        mynt::Vector3 u = R_kk_1 * IMUState::gravity;
        mynt::Vector3 s = (1.0 / u.dot(u)) * u;

//        mynt::Matrix A1 = Phi.block<3, 3>(6, 0);
        mynt::Matrix A1 = Phi.block<3, 3>(6, 0);
        mynt::Vector3 w1 = mynt::skew_symmetric(imu_state.velocity_null - imu_state.velocity) * IMUState::gravity;
        Phi.set_mat(6, 0, A1 - (A1 * u - w1) * s.transpose());

        mynt::Matrix A2 = Phi.block<3, 3>(12, 0);
        mynt::Vector3 w2 = mynt::skew_symmetric(dtime * imu_state.velocity_null + imu_state.position_null - imu_state.position) * IMUState::gravity;
        Phi.set_mat(12, 0, A2 - (A2 * u - w2) * s.transpose());

        // Propogate the state covariance matrix.
        mynt::Matrix Q = Phi * G * state_server.continuous_noise_cov * G.transpose() * Phi.transpose() * dtime;
        state_server.state_cov.set_mat(0, 0, Phi * state_server.state_cov.block<21, 21>(0, 0) * Phi.transpose() + Q);

        if (state_server.cam_states.size() > 0) {
            state_server.state_cov.set_mat(0, 21,
                    Phi * state_server.state_cov.block(0, 21, 21, state_server.state_cov.cols() - 21));
            state_server.state_cov.set_mat(21, 0,
                    state_server.state_cov.block(21, 0, state_server.state_cov.rows() - 21, 21) * Phi.transpose());
        }

        mynt::Matrix state_cov_fixed = (state_server.state_cov + state_server.state_cov.transpose()) * 0.5;
        state_server.state_cov = state_cov_fixed;

        // Update the state correspondes to null space.
        imu_state.orientation_null = imu_state.orientation;
        imu_state.position_null = imu_state.position;
        imu_state.velocity_null = imu_state.velocity;

        // Update the state info
        state_server.imu_state.time = time;

        return;
    }

    void MsckfVio::predictNewState(const double &dt, const mynt::Vector3 &gyro, const mynt::Vector3 &acc) {
        // TODO: Will performing the forward integration using the inverse of the quaternion give better accuracy?
        double gyro_norm = gyro.l2norm();
        mynt::Matrix Omega(4, 4);
        Omega.set_mat(0, 0, -mynt::skew_symmetric(gyro));
        Omega.set_mat(0, 3,  gyro);
        Omega.set_mat(3, 0, -gyro.transpose());

        mynt::Quarternion &q = state_server.imu_state.orientation;
        mynt::Vector3 &v = state_server.imu_state.velocity;
        mynt::Vector3 &p = state_server.imu_state.position;

        // Some pre-calculation
        mynt::Quarternion dq_dt, dq_dt2;
        if (gyro_norm > 1e-5) {
            dq_dt  = mynt::Vector4((cos(gyro_norm * dt * 0.5)  * mynt::Matrix::eye(4) + 1 / gyro_norm * sin(gyro_norm * dt * 0.5)  * Omega) * q.vector4());
            dq_dt2 = mynt::Vector4((cos(gyro_norm * dt * 0.25) * mynt::Matrix::eye(4) + 1 / gyro_norm * sin(gyro_norm * dt * 0.25) * Omega) * q.vector4());
        } else {
            dq_dt  = mynt::Vector4((mynt::Matrix::eye(4) + 0.5  * dt * Omega) * cos(gyro_norm * dt * 0.5)  * q.vector4());
            dq_dt2 = mynt::Vector4((mynt::Matrix::eye(4) + 0.25 * dt * Omega) * cos(gyro_norm * dt * 0.25) * q.vector4());
        }
        mynt::RotationMatrix dR_dt_transpose  =  dq_dt.rotation_matrix().transpose();
        mynt::RotationMatrix dR_dt2_transpose = dq_dt2.rotation_matrix().transpose();

        // k1 = f(tn, yn)
        mynt::Vector3 k1_v_dot = q.rotation_matrix().transpose() * acc + IMUState::gravity;
        mynt::Vector3 k1_p_dot = v;

        // k2 = f(tn+dt/2, yn+k1*dt/2)
        mynt::Vector3 k1_v = v + k1_v_dot * dt / 2;
        mynt::Vector3 k2_v_dot = dR_dt2_transpose * acc + IMUState::gravity;
        mynt::Vector3 k2_p_dot = k1_v;

        // k3 = f(tn+dt/2, yn+k2*dt/2)
        mynt::Vector3 k2_v = v + k2_v_dot * dt / 2;
        mynt::Vector3 k3_v_dot = dR_dt2_transpose * acc + IMUState::gravity;
        mynt::Vector3 k3_p_dot = k2_v;

        // k4 = f(tn+dt, yn+k3*dt)
        mynt::Vector3 k3_v = v + k3_v_dot * dt;
        mynt::Vector3 k4_v_dot = dR_dt_transpose * acc + IMUState::gravity;
        mynt::Vector3 k4_p_dot = k3_v;

        // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
        q = dq_dt.normalized();
        v = v + dt / 6 * (k1_v_dot + 2 * k2_v_dot + 2 * k3_v_dot + k4_v_dot);
        p = p + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

        return;
    }

    void MsckfVio::stateAugmentation(const double &time) {
        const mynt::RotationMatrix &R_i_c = state_server.imu_state.R_imu_cam0;
        const mynt::Vector3 &t_c_i = state_server.imu_state.t_cam0_imu;

        // Add a new camera state to the state server.
        mynt::RotationMatrix R_w_i = state_server.imu_state.orientation.rotation_matrix();
        mynt::RotationMatrix R_w_c = R_i_c * R_w_i;
        mynt::Vector3 t_c_w = state_server.imu_state.position + R_w_i.transpose() * t_c_i;

        state_server.cam_states[state_server.imu_state.id] = CAMState(state_server.imu_state.id);
        CAMState &cam_state = state_server.cam_states[state_server.imu_state.id];

        cam_state.time = time;
        cam_state.orientation = R_w_c.quarternion();
        cam_state.position = t_c_w;

        cam_state.orientation_null = cam_state.orientation;
        cam_state.position_null    = cam_state.position;

        // Update the covariance matrix of the state.
        // To simplify computation, the matrix J below is the nontrivial block
        // in Equation (16) in "A Multi-State Constraint Kalman Filter for Vision
        // -aided Inertial Navigation".
        mynt::Matrix J(6,21);
        J.set_mat(0, 0, R_i_c);
        J.set_mat(0, 15, mynt::Matrix::eye(3));
        J.set_mat(3, 0,  mynt::skew_symmetric(R_w_i.transpose() * t_c_i));
        //J.block<3, 3>(3, 0) = -R_w_i.transpose()*skewSymmetric(t_c_i);
        J.set_mat(3, 12, mynt::Matrix::eye(3));
        J.set_mat(3, 18, mynt::Matrix::eye(3));

        // Resize the state covariance matrix.
        size_t old_rows = state_server.state_cov.rows();
        size_t old_cols = state_server.state_cov.cols();
        state_server.state_cov.conservative_resize(old_rows + 6, old_cols + 6);

        // Rename some matrix blocks for convenience.
        const mynt::Matrix &P11 = state_server.state_cov.block<21, 21>(0, 0);
        const mynt::Matrix &P12 = state_server.state_cov.block(0, 21, 21, old_cols - 21);

        // Fill in the augmented state covariance.
//        state_server.state_cov.block(old_rows, 0, 6, old_cols) << J * P11, J * P12;
        state_server.state_cov.set_mat(old_rows, 0,  J * P11);
        state_server.state_cov.set_mat(old_rows, 21, J * P12);
        state_server.state_cov.set_mat(0, old_cols, state_server.state_cov.block(old_rows, 0, 6, old_cols).transpose());
        state_server.state_cov.set_mat(old_rows, old_cols, J * P11 * J.transpose());

        // Fix the covariance to be symmetric
        mynt::Matrix state_cov_fixed = (state_server.state_cov + state_server.state_cov.transpose()) / 2.0;
        state_server.state_cov = state_cov_fixed;

        return;
    }

    void MsckfVio::addFeatureObservations(const CameraMeasurementConstPtr &msg) {
        StateIDType state_id = state_server.imu_state.id;
        int curr_feature_num = map_server.size();
        int tracked_feature_num = 0;

        // Add new observations for existing features or new features in the map server.
        for (const auto &feature : msg->features) {
            if (map_server.find(feature.id) == map_server.end()) {
                // This is a new feature.
                map_server[feature.id] = Feature(feature.id);
                map_server[feature.id].observations[state_id] = mynt::Vector4({feature.u0, feature.v0, feature.u1, feature.v1});
            } else {
                // This is an old feature.
                map_server[feature.id].observations[state_id] = mynt::Vector4({feature.u0, feature.v0, feature.u1, feature.v1});
                ++tracked_feature_num;
            }
        }

        tracking_rate = static_cast<double>(tracked_feature_num) / static_cast<double>(curr_feature_num);

        return;
    }

    void MsckfVio::measurementJacobian(
            const StateIDType &cam_state_id, const FeatureIDType &feature_id,
            mynt::Matrix& H_x, mynt::Matrix& H_f, mynt::Vector4 &r) {

        // Prepare all the required data.
        const CAMState &cam_state = state_server.cam_states[cam_state_id];
        const Feature &feature = map_server[feature_id];

        // Cam0 pose.
        mynt::RotationMatrix R_w_c0 = cam_state.orientation.rotation_matrix();
        const mynt::Vector3 &t_c0_w = cam_state.position;

        // Cam1 pose.
        mynt::RotationMatrix R_c0_c1 = CAMState::T_cam0_cam1.rotation_matrix();
        mynt::RotationMatrix R_w_c1  = CAMState::T_cam0_cam1.rotation_matrix() * R_w_c0;
        mynt::Vector3 t_c1_w = t_c0_w - R_w_c1.transpose() * CAMState::T_cam0_cam1.translation();

        // 3d feature position in the world frame.
        // And its observation with the stereo cameras.
        const mynt::Vector3 &p_w = feature.position;
        const mynt::Vector4 &z = feature.observations.find(cam_state_id)->second;

        // Convert the feature position from the world frame to
        // the cam0 and cam1 frame.
        mynt::Vector3 p_c0 = R_w_c0 * (p_w - t_c0_w);
        mynt::Vector3 p_c1 = R_w_c1 * (p_w - t_c1_w);

        // Compute the Jacobians.
        mynt::Matrix dz_dpc0(4,3);
        dz_dpc0(0, 0) = 1 / p_c0[2];
        dz_dpc0(1, 1) = 1 / p_c0[2];
        dz_dpc0(0, 2) = -p_c0[0] / (p_c0[2] * p_c0[2]);
        dz_dpc0(1, 2) = -p_c0[1] / (p_c0[2] * p_c0[2]);

        mynt::Matrix dz_dpc1(4,3);
        dz_dpc1(2, 0) = 1 / p_c1[2];
        dz_dpc1(3, 1) = 1 / p_c1[2];
        dz_dpc1(2, 2) = -p_c1[0] / (p_c1[2] * p_c1[2]);
        dz_dpc1(3, 2) = -p_c1[1] / (p_c1[2] * p_c1[2]);

        mynt::Matrix dpc0_dxc(3,6);
        dpc0_dxc.set_mat(0, 0, mynt::skew_symmetric(p_c0));
        dpc0_dxc.set_mat(0, 3, -R_w_c0);

        mynt::Matrix dpc1_dxc(3,6);
        dpc1_dxc.set_mat(0, 0, R_c0_c1 * mynt::skew_symmetric(p_c0));
        dpc1_dxc.set_mat(0, 3, -R_w_c1);

        mynt::RotationMatrix dpc0_dpg = R_w_c0;
        mynt::RotationMatrix dpc1_dpg = R_w_c1;

        H_x = dz_dpc0 * dpc0_dxc + dz_dpc1 * dpc1_dxc;
        H_f = dz_dpc0 * dpc0_dpg + dz_dpc1 * dpc1_dpg;

        // Modifty the measurement Jacobian to ensure
        // observability constrain.
        mynt::Matrix A = H_x;
        mynt::Matrix u(6,1);
        u.set_mat(0, 0, cam_state.orientation_null.rotation_matrix() * IMUState::gravity);
        u.set_mat(3, 0, mynt::skew_symmetric(p_w - cam_state.position_null) * IMUState::gravity);
        H_x = A - A * u * (u.transpose() * u).inv() * u.transpose();
        H_f = -const_cast<const mynt::Matrix&>(H_x).block<4, 3>(0, 3);

        // Compute the residual.
        r = z - mynt::Vector4({p_c0[0] / p_c0[2], p_c0[1] / p_c0[2], p_c1[0] / p_c1[2], p_c1[1] / p_c1[2]});

        return;
    }

    void MsckfVio::featureJacobian(
            const FeatureIDType &feature_id, const std::vector<StateIDType> &cam_state_ids,
            mynt::Matrix &H_x, mynt::VectorX &r) {

        const auto &feature = map_server[feature_id];

        // Check how many camera states in the provided camera
        // id camera has actually seen this feature.
        std::vector<StateIDType> valid_cam_state_ids(0);
        for (const auto &cam_id : cam_state_ids) {
            if (feature.observations.find(cam_id) == feature.observations.end())
                continue;

            valid_cam_state_ids.push_back(cam_id);
        }

        int jacobian_row_size = 0;
        jacobian_row_size = 4 * valid_cam_state_ids.size();

        mynt::Matrix H_xj(jacobian_row_size, 21 + state_server.cam_states.size() * 6);
        mynt::Matrix H_fj(jacobian_row_size, 3);
        mynt::VectorX r_j(jacobian_row_size);
        int stack_cntr = 0;

        for (const auto &cam_id : valid_cam_state_ids) {
            mynt::Matrix H_xi(4,6);
            mynt::Matrix H_fi(4,3);
            mynt::Vector4 r_i;
            measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

                auto cam_state_iter = state_server.cam_states.find(cam_id);
            int cam_state_cntr = std::distance(state_server.cam_states.begin(), cam_state_iter);

            // Stack the Jacobians.
            H_xj.set_mat(stack_cntr, 21 + 6 * cam_state_cntr, H_xi);
            H_fj.set_mat(stack_cntr, 0, H_fi);
            r_j.set_segment(stack_cntr, r_i);
            stack_cntr += 4;
        }

        if(n_pub == 9) {
            debug_ << "featureJacobian H_xj:\n" << H_xj << std::endl;
            debug_ << "featureJacobian H_fj:\n" << H_fj << std::endl;
            debug_ << "featureJacobian r_j:\n" << r_j << std::endl;
        }

        // Project the residual and Jacobians onto the nullspace of H_fj.

        // TODO: verify

        /// Eigen
//        Eigen::MatrixXd eH_fj(H_fj.rows(), H_fj.cols());
//        for(int i=0; i<H_fj.rows(); ++i)
//            for(int j=0; j<H_fj.cols(); ++j)
//                eH_fj(i, j) = H_fj(i, j);
//
//        Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(eH_fj, Eigen::ComputeFullU | Eigen::ComputeThinV);
//        Eigen::MatrixXd eA = svd_helper.matrixU().rightCols(jacobian_row_size - 3);
//
//        mynt::Matrix A(eA.rows(), eA.cols());
//        for(int i=0; i<eA.rows(); ++i)
//            for(int j=0; j<eA.cols(); ++j)
//                A(i, j) = eA(i, j);

        /// OpenCV
//        cv::Mat cvH(H_fj.rows(), H_fj.cols(), CV_32F);
//        for(int i=0; i<H_fj.rows(); ++i)
//            for(int j=0; j<H_fj.cols(); ++j)
//                cvH.at<float>(i, j) = H_fj(i, j);
//        cv::Mat cvS, cvU, cvVt;
//        cv::SVD::compute(cvH, cvS, cvU, cvVt, cv::SVD::FULL_UV); // cv::SVD::FULL_UV
//        int j_start = cvU.cols-(jacobian_row_size-3);
//        mynt::Matrix A(cvU.rows, jacobian_row_size - 3);
//        for(int i=0; i<cvU.rows; ++i)
//            for(int j=j_start; j<cvU.cols; ++j)
//                A(i, j-j_start) = cvU.at<float>(i, j);

        /// Shen according OpenCV
        mynt::Matrix U1, W1, Vt1;
        mynt::svd_fulluv(H_fj, W1, U1, Vt1);
        int j_start = U1.cols()-(jacobian_row_size-3);
        mynt::Matrix A(U1.rows(), jacobian_row_size - 3);
        for(int i=0; i<U1.rows(); ++i)
            for(int j=j_start; j<U1.cols(); ++j)
                A(i, j-j_start) = U1(i, j);

        H_x = A.transpose() * H_xj;
        r   = A.transpose() * r_j;

//        if(n_pub == 9) {
//            debug_ << "featureJacobian U:\n" << svd_helper.matrixU() << std::endl;
//            debug_ << "featureJacobian H_x:\n" << H_x << std::endl;
//            debug_ << "featureJacobian r:\n"   << r << std::endl;
//        }

        return;
    }

    // TODO
    void MsckfVio::measurementUpdate(const mynt::Matrix &H, const mynt::VectorX &r) {
        if (H.rows() == 0 || r.rows() == 0)
            return;

        /// temp
        Eigen::MatrixXd He(H.rows(), H.cols());
        for(int i=0; i<H.rows(); ++i)
            for(int j=0; j<H.cols(); ++j)
                He(i,j) = H(i,j);
        Eigen::VectorXd re(r.size());
        for(int i=0; i<r.size(); ++i)
            re[i] = r[i];

        // Decompose the final Jacobian matrix to reduce computational complexity as in Equation (28), (29).
        Eigen::MatrixXd eH_thin;
        Eigen::VectorXd er_thin;

        if (H.rows() > H.cols()) {
            // Convert H to a sparse matrix.
            Eigen::SparseMatrix<double> H_sparse = He.sparseView();

            // Perform QR decompostion on H_sparse.
            Eigen::SPQR<Eigen::SparseMatrix<double> > spqr_helper;
            spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL); // SPQR_ORDERING_NATURAL from suitesparse
            spqr_helper.compute(H_sparse);

            Eigen::MatrixXd H_temp;
            Eigen::VectorXd r_temp;
            (spqr_helper.matrixQ().transpose() * He).evalTo(H_temp);
            (spqr_helper.matrixQ().transpose() * re).evalTo(r_temp);

            eH_thin = H_temp.topRows(21 + state_server.cam_states.size() * 6);
            er_thin = r_temp.head(21    + state_server.cam_states.size() * 6);

//            HouseholderQR<MatrixXd> qr_helper(H);
//            MatrixXd Q = qr_helper.householderQ();
//            MatrixXd Q1 = Q.leftCols(21+state_server.cam_states.size()*6);
//
//            H_thin = Q1.transpose() * H;
//            r_thin = Q1.transpose() * r;
        } else {
            eH_thin = He;
            er_thin = re;
        }

        mynt::Matrix H_thin(eH_thin.rows(), eH_thin.cols());
        for(int i=0; i<eH_thin.rows(); ++i)
            for(int j=0; j<eH_thin.cols(); ++j)
                H_thin(i,j) = eH_thin(i,j);
        mynt::VectorX r_thin(er_thin.size());
        for(int i=0; i<er_thin.size(); ++i)
            r_thin[i] = er_thin(i);

        // Compute the Kalman gain.
        const mynt::Matrix &P = state_server.state_cov;
        mynt::Matrix S = H_thin * P * H_thin.transpose() + Feature::observation_noise * mynt::Matrix::identity(H_thin.rows(), H_thin.rows());
        // MatrixXd K_transpose = S.fullPivHouseholderQr().solve(H_thin*P);

        // TODO: verify

        Eigen::MatrixXd eS(S.rows(), S.cols());
        for(int i=0; i<S.rows(); ++i)
            for(int j=0; j<S.cols(); ++j)
                eS(i,j) = S(i,j);
        Eigen::MatrixXd eP(P.rows(), P.cols());
        for(int i=0; i<P.rows(); ++i)
            for(int j=0; j<P.cols(); ++j)
                eP(i,j) = P(i,j);
        Eigen::VectorXd er(r.size());
        for(int i=0; i<r.size(); ++i)
            er[i] = r[i];

        Eigen::MatrixXd eK_transpose = eS.ldlt().solve(eH_thin * eP);
        Eigen::MatrixXd eK = eK_transpose.transpose();

        // TODO
        mynt::Matrix K(eK.rows(), eK.cols());
        for(int i=0; i<eK.rows(); ++i)
            for(int j=0; j<eK.cols(); ++j)
                K(i,j) = eK(i,j);

        // Compute the error of the state.
        mynt::VectorX delta_x = K * r_thin;

        // Update the IMU state.
        const mynt::VectorX &delta_x_imu = delta_x.head<21>();

        if (//delta_x_imu.segment<3>(0).norm() > 0.15 ||
            //delta_x_imu.segment<3>(3).norm() > 0.15 ||
                delta_x_imu.segment<3>(6).l2norm() > 0.5 ||
                //delta_x_imu.segment<3>(9).norm() > 0.5 ||
                delta_x_imu.segment<3>(12).l2norm() > 1.0) {
            printf("delta velocity: %f\n", delta_x_imu.segment<3>(6).l2norm());
            printf("delta position: %f\n", delta_x_imu.segment<3>(12).l2norm());
            std::cout << "Update change is too large." << std::endl;
            //return;
        }

        const mynt::Quarternion dq_imu = mynt::Quarternion::small_angle_quaternion(delta_x_imu.head<3>());
        state_server.imu_state.orientation = dq_imu * state_server.imu_state.orientation;
        state_server.imu_state.gyro_bias += delta_x_imu.segment<3>(3);
        state_server.imu_state.velocity  += delta_x_imu.segment<3>(6);
        state_server.imu_state.acc_bias  += delta_x_imu.segment<3>(9);
        state_server.imu_state.position  += delta_x_imu.segment<3>(12);

        const mynt::Quarternion dq_extrinsic = mynt::Quarternion::small_angle_quaternion(delta_x_imu.segment<3>(15));
        state_server.imu_state.R_imu_cam0 = dq_extrinsic.rotation_matrix() * state_server.imu_state.R_imu_cam0;
        state_server.imu_state.t_cam0_imu += delta_x_imu.segment<3>(18);

        // Update the camera states.
        auto cam_state_iter = state_server.cam_states.begin();
        for (int i = 0; i < state_server.cam_states.size(); ++i, ++cam_state_iter) {
            const mynt::VectorX &delta_x_cam = delta_x.segment<6>(21 + i * 6);
            const mynt::Quarternion dq_cam = mynt::Quarternion::small_angle_quaternion(delta_x_cam.head<3>());
            cam_state_iter->second.orientation = dq_cam * cam_state_iter->second.orientation;
            cam_state_iter->second.position += delta_x_cam.tail<3>();
        }

        // Update state covariance.
        mynt::Matrix I_KH = mynt::Matrix::identity(K.rows(), H_thin.cols()) - K * H_thin;
        //state_server.state_cov = I_KH*state_server.state_cov*I_KH.transpose() +
        //  K*K.transpose()*Feature::observation_noise;
        state_server.state_cov = I_KH * state_server.state_cov;

        // Fix the covariance to be symmetric
        mynt::Matrix state_cov_fixed = (state_server.state_cov + state_server.state_cov.transpose()) / 2.0;
        state_server.state_cov = state_cov_fixed;

        return;
    }

    bool MsckfVio::gatingTest(const mynt::Matrix &H, const mynt::VectorX &r, const int &dof) {
        mynt::Matrix P1 = H * state_server.state_cov * H.transpose();
        mynt::Matrix P2 = Feature::observation_noise * mynt::Matrix::eye(H.rows());
        mynt::Matrix P = P1 + P2;

        // TODO: verify

        Eigen::MatrixXd eP(P.rows(), P.cols());
        for(int i=0; i<P.rows(); ++i)
            for(int j=0; j<P.cols(); ++j)
                eP(i,j) = P(i,j);
        Eigen::VectorXd er(r.size());
        for(int i=0; i<r.size(); ++i)
            er[i] = r[i];

        double gamma = er.transpose() * eP.ldlt().solve(er);

        // cout << dof << " " << gamma << " " << chi_squared_test_table[dof] << " ";

        if (gamma < chi_squared_test_table[dof]) {
            //cout << "passed" << endl;
            return true;
        } else {
            //cout << "failed" << endl;
            return false;
        }
    }

    void MsckfVio::removeLostFeatures() {
        // Remove the features that lost track.
        // BTW, find the size the final Jacobian matrix and residual vector.
        int jacobian_row_size = 0;
        std::vector<FeatureIDType> invalid_feature_ids(0);
        std::vector<FeatureIDType> processed_feature_ids(0);

        int n = 0;
        for (auto iter = map_server.begin(); iter != map_server.end(); ++iter, ++n) {
            // Rename the feature to be checked.
            auto &feature = iter->second;

            // Pass the features that are still being tracked.
            if (feature.observations.find(state_server.imu_state.id) != feature.observations.end())
                continue;
            if (feature.observations.size() < 3) {
                invalid_feature_ids.push_back(feature.id);
                continue;
            }

            // Check if the feature can be initialized if it has not been.
            if (!feature.is_initialized) {
                if (!feature.checkMotion(state_server.cam_states)) {
                    invalid_feature_ids.push_back(feature.id);
                    continue;
                } else {
                    if (!feature.initializePosition(state_server.cam_states)) {
                        invalid_feature_ids.push_back(feature.id);
                        continue;
                    }
                }
            }

            jacobian_row_size += 4 * feature.observations.size() - 3;
            processed_feature_ids.push_back(feature.id);
        }

        //cout << "invalid/processed feature #: " <<
        //  invalid_feature_ids.size() << "/" <<
        //  processed_feature_ids.size() << endl;
        //cout << "jacobian row #: " << jacobian_row_size << endl;

        // Remove the features that do not have enough measurements.
        for (const auto &feature_id : invalid_feature_ids)
            map_server.erase(feature_id);

        // Return if there is no lost feature to be processed.
        if (processed_feature_ids.size() == 0) return;

        mynt::Matrix H_x(jacobian_row_size, 21 + 6 * state_server.cam_states.size());
        mynt::VectorX r(jacobian_row_size);
        int stack_cntr = 0;

        // Process the features which lose track.
        for (const auto &feature_id : processed_feature_ids) {
            auto &feature = map_server[feature_id];

            std::vector<StateIDType> cam_state_ids(0);
            for (const auto &measurement : feature.observations)
                cam_state_ids.push_back(measurement.first);

            mynt::Matrix H_xj;
            mynt::VectorX r_j;
            featureJacobian(feature.id, cam_state_ids, H_xj, r_j);

            if (gatingTest(H_xj, r_j, cam_state_ids.size() - 1)) {
                H_x.set_mat(stack_cntr, 0, H_xj);
                r.set_segment(stack_cntr, r_j);
                stack_cntr += H_xj.rows();
            }

            // Put an upper bound on the row size of measurement Jacobian, which helps guarantee the executation time.
            if (stack_cntr > 1500)
                break;
        }

        H_x.conservative_resize(stack_cntr, H_x.cols());
        r.conservative_resize(stack_cntr);

        // Perform the measurement update step.
        measurementUpdate(H_x, r);

        // Remove all processed features from the map.
        for (const auto &feature_id : processed_feature_ids)
            map_server.erase(feature_id);

        return;
    }

    void MsckfVio::findRedundantCamStates(std::vector<StateIDType> &rm_cam_state_ids) {
        // Move the iterator to the key position.
        auto key_cam_state_iter = state_server.cam_states.end();
        for (int i = 0; i < 4; ++i)
            --key_cam_state_iter;
        auto cam_state_iter = key_cam_state_iter;
        ++cam_state_iter;
        auto first_cam_state_iter = state_server.cam_states.begin();

        // Pose of the key camera state.
        const mynt::Vector3 key_position = key_cam_state_iter->second.position;
        const mynt::RotationMatrix key_rotation = key_cam_state_iter->second.orientation.rotation_matrix();

        // Mark the camera states to be removed based on the
        // motion between states.
        for (int i = 0; i < 2; ++i) {
            const mynt::Vector3 position = cam_state_iter->second.position;
            const mynt::RotationMatrix rotation = cam_state_iter->second.orientation.rotation_matrix();

            double distance = (position - key_position).l2norm();
            mynt::RotationMatrix R = rotation * key_rotation.transpose();

            // TODO [cg]
            Eigen::Matrix3d m3;
            for(int i=0; i<3; ++i)
                for(int j=0; j<3; ++j)
                    m3(i,j) = R(i,j);

            double angle = Eigen::AngleAxisd(m3).angle();

            if (angle < rotation_threshold &&
                distance < translation_threshold &&
                tracking_rate > tracking_rate_threshold) {
                rm_cam_state_ids.push_back(cam_state_iter->first);
                ++cam_state_iter;
            } else {
                rm_cam_state_ids.push_back(first_cam_state_iter->first);
                ++first_cam_state_iter;
            }
        }

        // Sort the elements in the output vector.
        sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());

        return;
    }

    void MsckfVio::pruneCamStateBuffer() {
        if (state_server.cam_states.size() < max_cam_state_size)
            return;

        // Find two camera states to be removed.
        std::vector<StateIDType> rm_cam_state_ids(0);
        findRedundantCamStates(rm_cam_state_ids);

        // Find the size of the Jacobian matrix.
        int jacobian_row_size = 0;
        for (auto &item : map_server) {
            auto &feature = item.second;
            // Check how many camera states to be removed are associated
            // with this feature.
            std::vector<StateIDType> involved_cam_state_ids(0);
            for (const auto &cam_id : rm_cam_state_ids) {
                if (feature.observations.find(cam_id) != feature.observations.end())
                    involved_cam_state_ids.push_back(cam_id);
            }

            if (involved_cam_state_ids.size() == 0)
                continue;
            if (involved_cam_state_ids.size() == 1) {
                feature.observations.erase(involved_cam_state_ids[0]);
                continue;
            }

            if (!feature.is_initialized) {
                // Check if the feature can be initialize.
                if (!feature.checkMotion(state_server.cam_states)) {
                    // If the feature cannot be initialized, just remove
                    // the observations associated with the camera states
                    // to be removed.
                    for (const auto &cam_id : involved_cam_state_ids)
                        feature.observations.erase(cam_id);
                    continue;
                } else {
                    if (!feature.initializePosition(state_server.cam_states)) {
                        for (const auto &cam_id : involved_cam_state_ids)
                            feature.observations.erase(cam_id);
                        continue;
                    }
                }
            }

            jacobian_row_size += 4 * involved_cam_state_ids.size() - 3;
        }

        //cout << "jacobian row #: " << jacobian_row_size << endl;

        // Compute the Jacobian and residual.
        mynt::Matrix H_x(jacobian_row_size, 21 + 6 * state_server.cam_states.size());
        mynt::VectorX r(jacobian_row_size);
        int stack_cntr = 0;

        for (auto &item : map_server) {
            auto &feature = item.second;
            // Check how many camera states to be removed are associated
            // with this feature.
            std::vector<StateIDType> involved_cam_state_ids(0);
            for (const auto &cam_id : rm_cam_state_ids) {
                if (feature.observations.find(cam_id) != feature.observations.end())
                    involved_cam_state_ids.push_back(cam_id);
            }

            if (involved_cam_state_ids.size() == 0)
                continue;

            mynt::Matrix H_xj;
            mynt::VectorX r_j;
            featureJacobian(feature.id, involved_cam_state_ids, H_xj, r_j);

            if (gatingTest(H_xj, r_j, involved_cam_state_ids.size())) {
                H_x.set_mat(stack_cntr, 0, H_xj);
                r.set_segment(stack_cntr, r_j);
                stack_cntr += H_xj.rows();
            }

            for (const auto &cam_id : involved_cam_state_ids)
                feature.observations.erase(cam_id);
        }

        H_x.conservative_resize(stack_cntr, H_x.cols());
        r.conservative_resize(stack_cntr);

        // Perform measurement update.
        measurementUpdate(H_x, r);

        for (const auto &cam_id : rm_cam_state_ids) {
            int cam_sequence = std::distance(state_server.cam_states.begin(), state_server.cam_states.find(cam_id));
            int cam_state_start = 21 + 6 * cam_sequence;
            int cam_state_end = cam_state_start + 6;

            // Remove the corresponding rows and columns in the state covariance matrix.
            if (cam_state_end < state_server.state_cov.rows()) {
                state_server.state_cov.set_mat(cam_state_start, 0,
                        state_server.state_cov.block(cam_state_end, 0, state_server.state_cov.rows() - cam_state_end, state_server.state_cov.cols()));

                state_server.state_cov.set_mat(0, cam_state_start,
                        state_server.state_cov.block(0, cam_state_end, state_server.state_cov.rows(), state_server.state_cov.cols() - cam_state_end));

                state_server.state_cov.conservative_resize(state_server.state_cov.rows() - 6, state_server.state_cov.cols() - 6);
            } else {
                state_server.state_cov.conservative_resize(state_server.state_cov.rows() - 6, state_server.state_cov.cols() - 6);
            }

            // Remove this camera state in the state vector.
            state_server.cam_states.erase(cam_id);
        }

        return;
    }

    void MsckfVio::onlineReset() {
        // Never perform online reset if position std threshold is non-positive.
        if (position_std_threshold <= 0)
            return;
        static long long int online_reset_counter = 0;

        // Check the uncertainty of positions to determine if
        // the system can be reset.
        double position_x_std = std::sqrt(state_server.state_cov(12, 12));
        double position_y_std = std::sqrt(state_server.state_cov(13, 13));
        double position_z_std = std::sqrt(state_server.state_cov(14, 14));

        if (position_x_std < position_std_threshold &&
            position_y_std < position_std_threshold &&
            position_z_std < position_std_threshold)
            return;

        printf("Start %lld online reset procedure...\n", ++online_reset_counter);
        printf("Stardard deviation in xyz: %f, %f, %f\n", position_x_std, position_y_std, position_z_std);

        // Remove all existing camera states.
        state_server.cam_states.clear();

        // Clear all exsiting features in the map.
        map_server.clear();

        // Reset the state covariance.
        YAML::Node cfg_msckfvio = YAML::LoadFile("../config/app_msckfvio.yaml");
        double gyro_bias_cov, acc_bias_cov, velocity_cov;
        velocity_cov  = cfg_msckfvio["initial_covariance/velocity"].as<double>();
        gyro_bias_cov = cfg_msckfvio["initial_covariance/gyro_bias"].as<double>();
        acc_bias_cov  = cfg_msckfvio["initial_covariance/acc_bias"].as<double>();
        double extrinsic_rotation_cov, extrinsic_translation_cov;
        extrinsic_rotation_cov    = cfg_msckfvio["initial_covariance/extrinsic_rotation_cov"].as<double>();
        extrinsic_translation_cov = cfg_msckfvio["initial_covariance/extrinsic_translation_cov"].as<double>();

        state_server.state_cov = mynt::Matrix(21, 21);
        for (int i = 3; i < 6; ++i)
            state_server.state_cov(i, i) = gyro_bias_cov;
        for (int i = 6; i < 9; ++i)
            state_server.state_cov(i, i) = velocity_cov;
        for (int i = 9; i < 12; ++i)
            state_server.state_cov(i, i) = acc_bias_cov;
        for (int i = 15; i < 18; ++i)
            state_server.state_cov(i, i) = extrinsic_rotation_cov;
        for (int i = 18; i < 21; ++i)
            state_server.state_cov(i, i) = extrinsic_translation_cov;

        printf("%lld online reset complete...\n", online_reset_counter);
        return;
    }

    void MsckfVio::publish(double time_stamp) {
        // Convert the IMU frame to the body frame.
        const IMUState &imu_state = state_server.imu_state;

        mynt::EuclideanTransform T_i_w;
        T_i_w.set_rotation_matrix(imu_state.orientation.rotation_matrix().transpose());
        T_i_w.set_translation(imu_state.position);

        mynt::EuclideanTransform T_b_w = IMUState::T_imu_body * T_i_w * IMUState::T_imu_body.inv();

        // Publish the odometry: T_b_w
        mynt::RotationMatrix m3_r = T_b_w.rotation_matrix();
        mynt::Vector3 v3_t = T_b_w.translation();
        mynt::Quarternion q4_r = m3_r.quarternion_hamilton(); // Hamilton for draw

        path_.push_back(v3_t);

        // TUM format
        pose_outfile_ << std::fixed << time_stamp << " "
                      << v3_t[0] << " " << v3_t[1] << " " << v3_t[2] << " "
                      << q4_r.x() << " " << q4_r.y() << " " << q4_r.z() << " " << q4_r.w() << std::endl;

        std::cout << std::fixed << time_stamp << " " << v3_t[0] << " " << v3_t[1] << " " << v3_t[2] << std::endl;

        // Convert the covariance.
        mynt::Matrix P_oo = const_cast<const mynt::Matrix&>(state_server.state_cov).block<3, 3>(0, 0);
        mynt::Matrix P_op = const_cast<const mynt::Matrix&>(state_server.state_cov).block<3, 3>(0, 12);
        mynt::Matrix P_po = const_cast<const mynt::Matrix&>(state_server.state_cov).block<3, 3>(12, 0);
        mynt::Matrix P_pp = const_cast<const mynt::Matrix&>(state_server.state_cov).block<3, 3>(12, 12);

        // TODO: verify
        mynt::Matrix P_imu_pose(6,6);
        P_imu_pose.set_mat(0, 0, P_pp);
        P_imu_pose.set_mat(0, 3, P_po);
        P_imu_pose.set_mat(3, 0, P_op);
        P_imu_pose.set_mat(3, 3, P_oo);

        mynt::Matrix H_pose(6,6);
        H_pose.set_mat(0, 0, IMUState::T_imu_body.rotation_matrix());
        H_pose.set_mat(3, 3, IMUState::T_imu_body.rotation_matrix());

        mynt::Matrix P_body_pose = H_pose * P_imu_pose * H_pose.transpose();

        std::vector<double> covariance_pose(36,0);
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                covariance_pose[6 * i + j] = P_body_pose(i, j);

        // Construct the covariance for the velocity.
        std::vector<double> covariance_twist(36,0);
        mynt::Matrix P_imu_vel = const_cast<const mynt::Matrix&>(state_server.state_cov).block<3, 3>(6, 6);
        mynt::Matrix H_vel = IMUState::T_imu_body.rotation_matrix();
        mynt::Matrix P_body_vel = H_vel * P_imu_vel * H_vel.transpose();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                covariance_twist[i * 6 + j] = P_body_vel(i, j);

        // Publish the 3D positions of the features that has been initialized.
        for (const auto &item : map_server) {
            const auto &feature = item.second;
            if (feature.is_initialized) {
                mynt::Vector3 feature_position = IMUState::T_imu_body.rotation_matrix() * feature.position;
                points3d_.push_back(mynt::Point3f(feature_position[0], feature_position[1], feature_position[2]));
            }
        }

        return;
    }

} // namespace msckf_vio

