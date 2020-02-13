/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_FEATURE_H
#define MSCKF_VIO_FEATURE_H

#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>

#include "common/imu_state.h"
#include "common/cam_state.h"
#include "kinematics/transform.h"

namespace mynt {

    /*
     * @brief Feature Salient part of an image. Please refer
     *    to the Appendix of "A Multi-State Constraint Kalman
     *    Filter for Vision-aided Inertial Navigation" for how
     *    the 3d position of a feature is initialized.
     */
    struct Feature {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef long long int FeatureIDType;

        /*
         * @brief OptimizationConfig Configuration parameters for 3d feature position optimization.
         */
        struct OptimizationConfig {
            double translation_threshold;
            double huber_epsilon;
            double estimation_precision;
            double initial_damping;
            int outer_loop_max_iteration;
            int inner_loop_max_iteration;

            OptimizationConfig() :
                    translation_threshold(0.2),
                    huber_epsilon(0.01),
                    estimation_precision(5e-7),
                    initial_damping(1e-3),
                    outer_loop_max_iteration(10),
                    inner_loop_max_iteration(10) {
                return;
            }
        };

        // Constructors for the struct.
        Feature() : id(0), is_initialized(false) {}

        Feature(const FeatureIDType &new_id) : id(new_id), is_initialized(false) {}

        Feature &operator=(const Feature &rhs) {
            if(this == &rhs)
                return *this;
            id  = rhs.id;
            next_id = rhs.next_id;
            observations = rhs.observations;
            position = rhs.position;
            is_initialized = rhs.is_initialized;
            observation_noise = rhs.observation_noise;
            optimization_config = rhs.optimization_config;
            return *this;
        }

        /*
         * @brief cost Compute the cost of the camera observations
         * @param T_c0_c1 A rigid body transformation takes
         *    a vector in c0 frame to ci frame.
         * @param x The current estimation.
         * @param z The ith measurement of the feature j in ci frame.
         * @return e The cost of this observation.
         */
        inline void cost(const mynt::EuclideanTransform &T_c0_ci,
                         const mynt::Vector3 &x, const mynt::Vector2 &z,
                         double &e) const;

        /*
         * @brief jacobian Compute the Jacobian of the camera observation
         * @param T_c0_c1 A rigid body transformation takes a vector in c0 frame to ci frame.
         * @param x The current estimation.
         * @param z The actual measurement of the feature in ci frame.
         * @return J The computed Jacobian.
         * @return r The computed residual.
         * @return w Weight induced by huber kernel.
         */
        inline void jacobian(const mynt::EuclideanTransform &T_c0_ci,
                             const mynt::Vector3 &x, const mynt::Vector2 &z,
                             mynt::Matrix &J, mynt::Vector2 &r,
                             double &w) const;

        /*
         * @brief generateInitialGuess Compute the initial guess of the feature's 3d position using only two views.
         * @param T_c1_c2: A rigid body transformation taking a vector from c2 frame to c1 frame.
         * @param z1: feature observation in c1 frame.
         * @param z2: feature observation in c2 frame.
         * @return p: Computed feature position in c1 frame.
         */
        inline void generateInitialGuess(
                const mynt::EuclideanTransform &T_c1_c2,
                const mynt::Vector2 &z1,
                const mynt::Vector2 &z2,
                mynt::Vector3 &p);

        /*
         * @brief checkMotion Check the input camera poses to ensure
         *    there is enough translation to triangulate the feature positon.
         * @param cam_states : input camera poses.
         * @return True if the translation between the input camera poses is sufficient.
         */
        inline bool checkMotion(const CamStateServer &cam_states) const;

        /*
         * @brief InitializePosition Intialize the feature position
         *    based on all current available measurements.
         * @param cam_states: A map containing the camera poses with its
         *    ID as the associated key value.
         * @return The computed 3d position is used to set the position
         *    member variable. Note the resulted position is in world
         *    frame.
         * @return True if the estimated 3d position of the feature
         *    is valid.
         */
        inline bool initializePosition(const CamStateServer &cam_states);

        // An unique identifier for the feature.
        // In case of long time running, the variable
        // type of id is set to FeatureIDType in order
        // to avoid duplication.
        FeatureIDType id;

        // id for next feature
        static FeatureIDType next_id;

        // Store the observations of the features in the
        // state_id(key)-image_coordinates(value) manner.
//        std::map<StateIDType, Eigen::Vector4d, std::less<StateIDType>,
//                Eigen::aligned_allocator<std::pair<const StateIDType, Eigen::Vector4d> > > observations;

        std::map<StateIDType, mynt::Vector4, std::less<StateIDType> > observations;

        // 3d postion of the feature in the world frame.
        mynt::Vector3 position;

        // A indicator to show if the 3d postion of the feature
        // has been initialized or not.
        bool is_initialized;

        // Noise for a normalized feature measurement.
        static double observation_noise;

        // Optimization configuration for solving the 3d position.
        static OptimizationConfig optimization_config;
    };

    typedef Feature::FeatureIDType FeatureIDType;
    typedef std::map<FeatureIDType, Feature, std::less<int>,
            Eigen::aligned_allocator<
                    std::pair<const FeatureIDType, Feature> > > MapServer;


    void Feature::cost(const mynt::EuclideanTransform &T_c0_ci,
                       const mynt::Vector3 &x, const mynt::Vector2 &z,
                       double &e) const {
        // Compute hi1, hi2, and hi3 as Equation (37).
        const double &alpha = x[0];
        const double &beta  = x[1];
        const double &rho   = x[2];

        mynt::Vector3 h = T_c0_ci.rotation_matrix() * mynt::Vector3({alpha, beta, 1.0}) + rho * T_c0_ci.translation();
        double &h1 = h[0];
        double &h2 = h[1];
        double &h3 = h[2];

        // Predict the feature observation in ci frame.
        mynt::Vector2 z_hat({h1 / h3, h2 / h3});

        // Compute the residual.
        e = mynt::Vector2(z_hat - z).squared_l2norm();
        return;
    }

    void Feature::jacobian(const mynt::EuclideanTransform &T_c0_ci,
                           const mynt::Vector3 &x, const mynt::Vector2 &z,
                           mynt::Matrix &J, mynt::Vector2 &r,
                           double &w) const {

        // Compute hi1, hi2, and hi3 as Equation (37).
        const double &alpha = x[0];
        const double &beta  = x[1];
        const double &rho   = x[2];

        mynt::Vector3 h = T_c0_ci.rotation_matrix() * mynt::Vector3({alpha, beta, 1.0}) + rho * T_c0_ci.translation();
        double &h1 = h[0];
        double &h2 = h[1];
        double &h3 = h[2];

        // Compute the Jacobian.
        std::vector<int> v_idx = {0,1};

        mynt::Matrix W(3,3);
        W.set_mat(0, 0, T_c0_ci.rotation_matrix().extract_cols(v_idx));
        W.set_mat(0, 2, T_c0_ci.translation());

        J.set_mat(0, 0, 1 / h3 * W.row(0) - h1 / (h3 * h3) * W.row(2));
        J.set_mat(1, 0, 1 / h3 * W.row(1) - h2 / (h3 * h3) * W.row(2));

        // Compute the residual.
        mynt::Vector2 z_hat({h1 / h3, h2 / h3});
        r = z_hat - z;

        // Compute the weight based on the residual.
        double e = r.l2norm();
        if (e <= optimization_config.huber_epsilon)
            w = 1.0;
        else
            w = std::sqrt(2.0 * optimization_config.huber_epsilon / e);

        return;
    }

    void Feature::generateInitialGuess(
            const mynt::EuclideanTransform &T_c1_c2,
            const mynt::Vector2 &z1,
            const mynt::Vector2 &z2,
            mynt::Vector3 &p) {
        // Construct a least square problem to solve the depth.
        mynt::Vector3 m = T_c1_c2.rotation_matrix() * mynt::Vector3({z1[0], z1[1], 1.0});

        mynt::Vector2 A;
        A[0] = m[0] - z2[0] * m[2];
        A[1] = m[1] - z2[1] * m[2];

        mynt::Vector2 b;
        b[0] = z2[0] * T_c1_c2.translation()[2] - T_c1_c2.translation()[0];
        b[1] = z2[1] * T_c1_c2.translation()[2] - T_c1_c2.translation()[1];

        // Solve for the depth.
        // d = AtA * Atb
        double depth = ((A.transpose() * A).inv() * A.transpose() * b)(0,0);
        p[0] = z1[0] * depth;
        p[1] = z1[1] * depth;
        p[2] = depth;

        return;
    }

    bool Feature::checkMotion(const CamStateServer &cam_states) const {
        const StateIDType &first_cam_id = observations.begin()->first;
        const StateIDType &last_cam_id = (--observations.end())->first;

        mynt::EuclideanTransform first_cam_pose;
        first_cam_pose.set_rotation_matrix(cam_states.find(first_cam_id)->second.orientation.rotation_matrix().transpose());
        first_cam_pose.set_translation(cam_states.find(first_cam_id)->second.position);

        mynt::EuclideanTransform last_cam_pose;
        last_cam_pose.set_rotation_matrix(cam_states.find(last_cam_id)->second.orientation.rotation_matrix().transpose());
        last_cam_pose.set_translation(cam_states.find(last_cam_id)->second.position);

        // Get the direction of the feature when it is first observed.
        // This direction is represented in the world frame.
        mynt::Vector3 feature_direction({observations.begin()->second[0], observations.begin()->second[1], 1.0});
        feature_direction = feature_direction / feature_direction.l2norm();
        feature_direction = first_cam_pose.rotation_matrix() * feature_direction;

        // Compute the translation between the first frame
        // and the last frame. We assume the first frame and
        // the last frame will provide the largest motion to
        // speed up the checking process.
        mynt::Vector3 translation = last_cam_pose.translation() - first_cam_pose.translation();
        double parallel_translation = translation.dot(feature_direction);
        mynt::Vector3 orthogonal_translation = translation - parallel_translation * feature_direction;

        if (orthogonal_translation.l2norm() > optimization_config.translation_threshold)
            return true;
        else
            return false;
    }

    bool Feature::initializePosition(const CamStateServer &cam_states) {
        // Organize camera poses and feature observations properly.

//        std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d> > cam_poses(0);
//        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > measurements(0);

        std::vector<mynt::EuclideanTransform> cam_poses;
        std::vector<mynt::Vector2> measurements;

        for (auto &m : observations) {
            // TODO: This should be handled properly. Normally, the
            //    required camera states should all be available in
            //    the input cam_states buffer.
            auto cam_state_iter = cam_states.find(m.first);
            if (cam_state_iter == cam_states.end())
                continue;

            // Add the measurement.
            measurements.push_back(m.second.block<2>(0));
            measurements.push_back(m.second.block<2>(2));

            // This camera pose will take a vector from this camera frame to the world frame.
            mynt::EuclideanTransform cam0_pose;
            cam0_pose.set_rotation_matrix(cam_state_iter->second.orientation.rotation_matrix().transpose());
            cam0_pose.set_translation(cam_state_iter->second.position);

            mynt::EuclideanTransform cam1_pose;
            cam1_pose = cam0_pose * CAMState::T_cam0_cam1.inv();

            cam_poses.push_back(cam0_pose);
            cam_poses.push_back(cam1_pose);
        }

        // All camera poses should be modified such that it takes a
        // vector from the first camera frame in the buffer to this
        // camera frame.
        mynt::EuclideanTransform T_c0_w = cam_poses[0];
        for (auto &pose : cam_poses)
            pose = pose.inv() * T_c0_w;

        // Generate initial guess
        mynt::Vector3 initial_position;
        generateInitialGuess(cam_poses[cam_poses.size() - 1], measurements[0], measurements[measurements.size() - 1], initial_position);

        mynt::Vector3 solution(
                {initial_position[0] / initial_position[2],
                 initial_position[1] / initial_position[2],
                 1.0 / initial_position[2]}
                 );

        // Apply Levenberg-Marquart method to solve for the 3d position.
        double lambda = optimization_config.initial_damping;
        int inner_loop_cntr = 0;
        int outer_loop_cntr = 0;
        bool is_cost_reduced = false;
        double delta_norm = 0;

        // Compute the initial cost.
        double total_cost = 0.0;
        for (int i = 0; i < cam_poses.size(); ++i) {
            double this_cost = 0.0;
            cost(cam_poses[i], solution, measurements[i], this_cost);
            total_cost += this_cost;
        }

        // Outer loop.
        do {
            mynt::Matrix A(3,3);
            mynt::Vector3 b;

            for (int i = 0; i < cam_poses.size(); ++i) {
                mynt::Matrix J(2, 3);
                mynt::Vector2 r;
                double w;

                jacobian(cam_poses[i], solution, measurements[i], J, r, w);

                if (w == 1) {
                    A += J.transpose() * J;
                    b += J.transpose() * r;
                } else {
                    double w_square = w * w;
                    A += w_square * J.transpose() * J;
                    b += w_square * J.transpose() * r;
                }
            }

            // Inner loop.
            // Solve for the delta that can reduce the total cost.
            do {
                // TODO[cg]: verify
                mynt::Matrix damper = lambda * mynt::Matrix::eye(3);
//                mynt::Vector3 delta = (A + damper).ldlt().solve(b);
                mynt::Matrix A_tmp = A + damper;
                mynt::Matrix b_tmp = b;

//                b_tmp.solve(A_tmp);
//                mynt::Vector3 delta = b_tmp;

                Eigen::Matrix3d eA;
                for(int i=0; i<3; ++i)
                    for(int j=0; j<3; ++j)
                        eA(i,j) = A_tmp(i,j);
                Eigen::Vector3d eb;
                for(int i=0; i<3; ++i)
                    eb[i] = b[i];
                Eigen::Vector3d e_delta = eA.ldlt().solve(eb);
                mynt::Vector3 delta;
                for(int i=0; i<3; ++i)
                    delta[i] = e_delta[i];

                mynt::Vector3 new_solution = solution - delta;
                delta_norm = delta.l2norm();

                double new_cost = 0.0;
                for (int i = 0; i < cam_poses.size(); ++i) {
                    double this_cost = 0.0;
                    cost(cam_poses[i], new_solution, measurements[i], this_cost);
                    new_cost += this_cost;
                }

                if (new_cost < total_cost) {
                    is_cost_reduced = true;
                    solution = new_solution;
                    total_cost = new_cost;
                    lambda = lambda / 10 > 1e-10 ? lambda / 10 : 1e-10;
                } else {
                    is_cost_reduced = false;
                    lambda = lambda * 10 < 1e12 ? lambda * 10 : 1e12;
                }

            } while (inner_loop_cntr++ < optimization_config.inner_loop_max_iteration && !is_cost_reduced);

            inner_loop_cntr = 0;

        } while (outer_loop_cntr++ <
                 optimization_config.outer_loop_max_iteration &&
                 delta_norm > optimization_config.estimation_precision);

        // Covert the feature position from inverse depth
        // representation to its 3d coordinate.
        mynt::Vector3 final_position({solution[0] / solution[2], solution[1] / solution[2], 1.0 / solution[2]});

        // Check if the solution is valid. Make sure the feature
        // is in front of every camera frame observing it.
        bool is_valid_solution = true;
        for (const auto &pose : cam_poses) {
            mynt::Vector3 position = pose.rotation_matrix() * final_position + pose.translation();
            if (position[2] <= 0) {
                is_valid_solution = false;
                break;
            }
        }

        // Convert the feature position to the world frame.
        position = T_c0_w.rotation_matrix() * final_position + T_c0_w.translation();

        if (is_valid_solution)
            is_initialized = true;

        return is_valid_solution;
    }
} // namespace msckf_vio

#endif // MSCKF_VIO_FEATURE_H
