/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_IMAGE_PROCESSOR_H
#define MSCKF_VIO_IMAGE_PROCESSOR_H

#include <vector>
#include <map>
#include <fstream>

#include "common/data_msg.h"
#include "common/config_io.h"
#include "maths/vector.h"
#include "cv/types.h"
#include "cv/corner_detector.h"
#include "kinematics/rotation_matrix.h"

namespace mynt {

    /**
     * @brief ImageProcessor Detects and tracks features in image sequences.
     */
    class ImageProcessor {
    public:
        // Constructor
        ImageProcessor(YAML::Node cfg_cam_imu);

        // Disable copy and assign constructors.
        ImageProcessor(const ImageProcessor &) = delete;

        ImageProcessor operator=(const ImageProcessor &) = delete;

        // Destructor
        ~ImageProcessor();

        // Initialize the object.
        bool initialize();

        /**
        * @brief stereoCallback
        *    Callback function for the stereo images.
        * @param cam0_img left image.
        * @param cam1_img right image.
        */
        void stereoCallback(const mynt::Image &cam0_img, const mynt::Image &cam1_img, bool is_draw = false);

        /**
         * @brief imuCallback
         *    Callback function for the imu message.
         * @param msg IMU msg.
         */
        void imuCallback(const mynt::ImuConstPtr &msg);

        std::shared_ptr<CameraMeasurement> feature_msg_ptr_;

        /**
         * @brief FeatureIDType An alias for unsigned long long int.
         */
        typedef unsigned long long int FeatureIDType;

        std::vector<FeatureIDType> prev_ids_;
        std::map<FeatureIDType, mynt::Point2f> prev_cam0_points_;
        std::map<FeatureIDType, mynt::Point2f> prev_cam1_points_;
        std::map<FeatureIDType, mynt::Point2f> curr_cam0_points_;
        std::map<FeatureIDType, mynt::Point2f> curr_cam1_points_;

        /**
         * @brief ProcessorConfig Configuration parameters for
         *    feature detection and tracking.
         */
        struct ProcessorConfig {
            int grid_row;
            int grid_col;
            int grid_min_feature_num;
            int grid_max_feature_num;

            int pyramid_levels;
            int patch_size;
            int fast_threshold;
            int max_iteration;
            double track_precision;
            double ransac_threshold;
            double stereo_threshold;
        };

        ProcessorConfig processor_config;

        typedef std::shared_ptr<ImageProcessor> Ptr;
        typedef std::shared_ptr<const ImageProcessor> ConstPtr;

    private:
        /*
         * @brief FeatureMetaData Contains necessary information
         *    of a feature for easy access.
         */
        struct FeatureMetaData {
            FeatureIDType id;
            float response;
            int lifetime;
            mynt::Point2f cam0_point;
            mynt::Point2f cam1_point;
        };

        /*
         * @brief GridFeatures Organize features based on the grid
         *    they belong to. Note that the key is encoded by the
         *    grid index.
         */
        typedef std::map<int, std::vector<FeatureMetaData> > GridFeatures;

        /*
         * @brief keyPointCompareByResponse
         *    Compare two keypoints based on the response.
         */
        static bool keyPointCompareByResponse(
                const std::pair<mynt::Point2f, double> &pt1,
                const std::pair<mynt::Point2f, double> &pt2) {
            // Keypoint with higher response will be at the
            // beginning of the vector.
            return pt1.second > pt2.second;
        }

        /*
         * @brief featureCompareByResponse
         *    Compare two features based on the response.
         */
        static bool featureCompareByResponse(
                const FeatureMetaData &f1,
                const FeatureMetaData &f2) {
            // Features with higher response will be at the
            // beginning of the vector.
            return f1.response > f2.response;
        }

        /*
         * @brief featureCompareByLifetime
         *    Compare two features based on the lifetime.
         */
        static bool featureCompareByLifetime(
                const FeatureMetaData &f1,
                const FeatureMetaData &f2) {
            // Features with longer lifetime will be at the
            // beginning of the vector.
            return f1.lifetime > f2.lifetime;
        }

        /*
         * @brief loadParameters
         *    Load parameters from the parameter server.
         */
        bool loadParameters();

        /*
         * @initializeFirstFrame
         *    Initialize the image processing sequence, which is
         *    bascially detect new features on the first set of
         *    stereo images.
         */
        void initializeFirstFrame();

        /*
         * @brief trackFeatures
         *    Tracker features on the newly received stereo images.
         */
        void trackFeatures();

        /*
         * @addNewFeatures
         *    Detect new features on the image to ensure that the
         *    features are uniformly distributed on the image.
         */
        void addNewFeatures();

        /*
         * @brief pruneGridFeatures
         *    Remove some of the features of a grid in case there are
         *    too many features inside of that grid, which ensures the
         *    number of features within each grid is bounded.
         */
        void pruneGridFeatures();

        /*
         * @brief publish
         *    Publish the features on the current image including
         *    both the tracked and newly detected ones.
         */
        void publish();

        /*
         * @brief createImagePyramids
         *    Create image pyramids used for klt tracking.
         */
        void createImagePyramids();

        /*
         * @brief integrateImuData Integrates the IMU gyro readings
         *    between the two consecutive images, which is used for
         *    both tracking prediction and 2-point RANSAC.
         * @return cam0_R_p_c: a rotation matrix which takes a vector
         *    from previous cam0 frame to current cam0 frame.
         * @return cam1_R_p_c: a rotation matrix which takes a vector
         *    from previous cam1 frame to current cam1 frame.
         */
        void integrateImuData(mynt::RotationMatrix &cam0_R_p_c, mynt::RotationMatrix &cam1_R_p_c);

        /*
         * @brief predictFeatureTracking Compensates the rotation
         *    between consecutive camera frames so that feature
         *    tracking would be more robust and fast.
         * @param input_pts: features in the previous image to be tracked.
         * @param R_p_c: a rotation matrix takes a vector in the previous
         *    camera frame to the current camera frame.
         * @param intrinsics: intrinsic matrix of the camera.
         * @return compensated_pts: predicted locations of the features
         *    in the current image based on the provided rotation.
         *
         * Note that the input and output points are of pixel coordinates.
         */
        void predictFeatureTracking(
                const std::vector<mynt::Point2f> &input_pts,
                const mynt::RotationMatrix &R_p_c,
                const mynt::Vector4 &intrinsics,
                std::vector<mynt::Point2f> &compensated_pts);

        /*
         * @brief twoPointRansac Applies two point ransac algorithm
         *    to mark the inliers in the input set.
         * @param pts1: first set of points.
         * @param pts2: second set of points.
         * @param R_p_c: a rotation matrix takes a vector in the previous
         *    camera frame to the current camera frame.
         * @param intrinsics: intrinsics of the camera.
         * @param distortion_model: distortion model of the camera.
         * @param distortion_coeffs: distortion coefficients.
         * @param inlier_error: acceptable error to be considered as an inlier.
         * @param success_probability: the required probability of success.
         * @return inlier_flag: 1 for inliers and 0 for outliers.
         */
        void twoPointRansac(
                const std::vector<mynt::Point2f> &pts1,
                const std::vector<mynt::Point2f> &pts2,
                const mynt::RotationMatrix &R_p_c,
                const mynt::Vector4 &intrinsics,
                const std::string &distortion_model,
                const mynt::Vector4 &distortion_coeffs,
                const double &inlier_error,
                const double &success_probability,
                std::vector<int> &inlier_markers);

        void undistortPoints(
                const std::vector<mynt::Point2f> &pts_in,
                const mynt::Vector4 &intrinsics,
                const std::string &distortion_model,
                const mynt::Vector4 &distortion_coeffs,
                std::vector<mynt::Point2f> &pts_out,
                const mynt::Mat3 &rectification_matrix = mynt::Matrix::eye(3),
                const mynt::Vector4 &new_intrinsics = mynt::Vector4({1, 1, 0, 0}));

        void rescalePoints(std::vector<mynt::Point2f> &pts1, std::vector<mynt::Point2f> &pts2, float &scaling_factor);

        std::vector<mynt::Point2f> distortPoints(
                const std::vector<mynt::Point2f> &pts_in,
                const mynt::Vector4 &intrinsics,
                const std::string &distortion_model,
                const mynt::Vector4 &distortion_coeffs);

        /*
         * @brief stereoMatch Matches features with stereo image pairs.
         * @param cam0_points: points in the primary image.
         * @return cam1_points: points in the secondary image.
         * @return inlier_markers: 1 if the match is valid, 0 otherwise.
         */
        void stereoMatch(
                const std::vector<mynt::Point2f> &cam0_points,
                std::vector<mynt::Point2f> &cam1_points,
                std::vector<unsigned char> &inlier_markers);

        /*
         * @brief removeUnmarkedElements Remove the unmarked elements
         *    within a vector.
         * @param raw_vec: vector with outliers.
         * @param markers: 0 will represent a outlier, 1 will be an inlier.
         * @return refined_vec: a vector without outliers.
         *
         * Note that the order of the inliers in the raw_vec is perserved
         * in the refined_vec.
         */
        template<typename T>
        void removeUnmarkedElements(
                const std::vector<T> &raw_vec,
                const std::vector<unsigned char> &markers,
                std::vector<T> &refined_vec) {
            if (raw_vec.size() != markers.size()) {
                printf("The input size of raw_vec(%lu) and markers(%lu) does not match...", raw_vec.size(), markers.size());
            }
            for (int i = 0; i < markers.size(); ++i) {
                if (markers[i] == 0)
                    continue;
                refined_vec.push_back(raw_vec[i]);
            }
            return;
        }

        YAML::Node cfg_cam_imu_;

        // Indicate if this is the first image message.
        bool is_first_img;

        // ID for the next new feature.
        FeatureIDType next_feature_id;

        // Feature detector
//        cv::Ptr<cv::Feature2D> detector_ptr;
        CornerDetector detector_;

        // IMU message buffer.
        std::vector<mynt::Imu> imu_msg_buffer;

        // Camera calibration parameters
        std::string cam0_distortion_model;
        mynt::Vector4 cam0_intrinsics;
        mynt::Vector4 cam0_distortion_coeffs;

        std::string cam1_distortion_model;
        mynt::Vector4 cam1_intrinsics;
        mynt::Vector4 cam1_distortion_coeffs;

        // Take a vector from cam0 frame to the IMU frame.
        mynt::RotationMatrix R_cam0_imu;
        mynt::Vector3 t_cam0_imu;
        // Take a vector from cam1 frame to the IMU frame.
        mynt::RotationMatrix R_cam1_imu;
        mynt::Vector3 t_cam1_imu;

        // Previous and current images
        std::shared_ptr<mynt::Image> cam0_prev_img_ptr;
        std::shared_ptr<mynt::Image> cam0_curr_img_ptr;
        std::shared_ptr<mynt::Image> cam1_curr_img_ptr;

        // Pyramids for previous and current image
        std::vector<mynt::YImg8> prev_cam0_pyramid_;
        std::vector<mynt::YImg8> curr_cam0_pyramid_;
        std::vector<mynt::YImg8> curr_cam1_pyramid_;

        // Features in the previous and current image.
        std::shared_ptr<GridFeatures> prev_features_ptr;
        std::shared_ptr<GridFeatures> curr_features_ptr;

        // Number of features after each outlier removal step.
        int before_tracking = 0;
        int after_tracking = 0;
        int after_matching = 0;
        int after_ransac = 0;

        // Debugging
        std::map<FeatureIDType, int> feature_lifetime;

        std::ofstream debug_;

        void updateFeatureLifetime();

        void featureLifetimeStatistics();
    };

    typedef ImageProcessor::Ptr ImageProcessorPtr;
    typedef ImageProcessor::ConstPtr ImageProcessorConstPtr;

} // end namespace msckf_vio

#endif
