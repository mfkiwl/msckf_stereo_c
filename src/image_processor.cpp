/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include "image_processor.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <set>

#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>

#include "maths/math_basics.h"
#include "kinematics/convertor.h"
#include "kinematics/transform.h"
#include "cv/undistort.h"
#include "cv/calib3d.h"
#include "cv/visual_tracking.h"

namespace mynt {

    ImageProcessor::ImageProcessor(YAML::Node cfg_cam_imu) :
            cfg_cam_imu_(cfg_cam_imu),
            feature_msg_ptr_(new CameraMeasurement),
            is_first_img(true),
            cam0_prev_img_ptr(new mynt::Image),
            cam0_curr_img_ptr(new mynt::Image),
            cam1_curr_img_ptr(new mynt::Image),
            prev_features_ptr(new GridFeatures()),
            curr_features_ptr(new GridFeatures()) {
        return;
    }

    ImageProcessor::~ImageProcessor() {
        //destroyAllWindows();
        //ROS_INFO("Feature lifetime statistics:");
        //featureLifetimeStatistics();
        debug_.close();
        return;
    }

    bool ImageProcessor::loadParameters() {
        // Camera calibration parameters
        cam0_distortion_model = cfg_cam_imu_["cam0"]["distortion_model"].as<std::string>();
        cam1_distortion_model = cfg_cam_imu_["cam1"]["distortion_model"].as<std::string>();

        cam0_intrinsics = cfg_cam_imu_["cam0"]["intrinsics"].as<mynt::Vector4>();
        cam1_intrinsics = cfg_cam_imu_["cam1"]["intrinsics"].as<mynt::Vector4>();

        cam0_distortion_coeffs = cfg_cam_imu_["cam0"]["distortion_coeffs"].as<mynt::Vector4>();
        cam1_distortion_coeffs = cfg_cam_imu_["cam1"]["distortion_coeffs"].as<mynt::Vector4>();

        mynt::EuclideanTransform m4_cam0_imu = cfg_cam_imu_["cam0"]["T_cam_imu"].as<mynt::Mat4>();

        R_cam0_imu = m4_cam0_imu.rotation_matrix().transpose(); // 坐标系变换
        t_cam0_imu = -R_cam0_imu * m4_cam0_imu.translation();

        mynt::EuclideanTransform m4_cam1_cam0 = cfg_cam_imu_["cam1"]["T_cn_cnm1"].as<mynt::Mat4>();

        mynt::EuclideanTransform T_cam1_imu = m4_cam1_cam0 * m4_cam0_imu;
        R_cam1_imu = T_cam1_imu.rotation_matrix().transpose();
        t_cam1_imu = -R_cam1_imu * T_cam1_imu.translation();

        // Processor parameters
        YAML::Node cfg_imgproc = YAML::LoadFile("../config/app_imgproc.yaml");
        processor_config.grid_row = cfg_imgproc["grid_row"].as<int>();
        processor_config.grid_col = cfg_imgproc["grid_col"].as<int>();
        processor_config.grid_min_feature_num = cfg_imgproc["grid_min_feature_num"].as<int>();
        processor_config.grid_max_feature_num = cfg_imgproc["grid_max_feature_num"].as<int>();
        processor_config.pyramid_levels = cfg_imgproc["pyramid_levels"].as<int>();
        processor_config.patch_size = cfg_imgproc["patch_size"].as<int>();
        processor_config.fast_threshold = cfg_imgproc["fast_threshold"].as<int>();
        processor_config.ransac_threshold = cfg_imgproc["ransac_threshold"].as<int>();
        processor_config.stereo_threshold = cfg_imgproc["stereo_threshold"].as<int>();
        processor_config.max_iteration = cfg_imgproc["max_iteration"].as<int>();
        processor_config.track_precision = cfg_imgproc["track_precision"].as<double>();

        printf("ImageProcessor begin ===========================================\n");
        
        printf("cam0_intrinscs: %f, %f, %f, %f\n", cam0_intrinsics[0], cam0_intrinsics[1], cam0_intrinsics[2], cam0_intrinsics[3]);
        printf("cam0_distortion_model: %s\n", cam0_distortion_model.c_str());
        printf("cam0_distortion_coefficients: %f, %f, %f, %f\n",
            cam0_distortion_coeffs[0], cam0_distortion_coeffs[1],
            cam0_distortion_coeffs[2], cam0_distortion_coeffs[3]);

        printf("cam1_intrinscs: %f, %f, %f, %f\n", cam1_intrinsics[0], cam1_intrinsics[1], cam1_intrinsics[2], cam1_intrinsics[3]);
        printf("cam1_distortion_model: %s\n", cam1_distortion_model.c_str());
        printf("cam1_distortion_coefficients: %f, %f, %f, %f\n",
            cam1_distortion_coeffs[0], cam1_distortion_coeffs[1],
            cam1_distortion_coeffs[2], cam1_distortion_coeffs[3]);

        printf("grid_row: %d\n", processor_config.grid_row);
        printf("grid_col: %d\n", processor_config.grid_col);
        printf("grid_min_feature_num: %d\n", processor_config.grid_min_feature_num);
        printf("grid_max_feature_num: %d\n", processor_config.grid_max_feature_num);
        printf("pyramid_levels: %d\n", processor_config.pyramid_levels);
        printf("patch_size: %d\n", processor_config.patch_size);
        printf("fast_threshold: %d\n", processor_config.fast_threshold);
        printf("max_iteration: %d\n", processor_config.max_iteration);
        printf("track_precision: %f\n", processor_config.track_precision);
        printf("ransac_threshold: %f\n", processor_config.ransac_threshold);
        printf("stereo_threshold: %f\n", processor_config.stereo_threshold);

        std::cout << "T_imu_cam0:\n"  << m4_cam0_imu  << std::endl;
        std::cout << "T_cam0_cam1:\n" << m4_cam1_cam0 << std::endl;
        std::cout << "R_cam0_imu:\n" << R_cam0_imu << std::endl;
        std::cout << "t_cam0_imu:\n" << t_cam0_imu << std::endl;
        std::cout << "R_cam1_imu:\n" << R_cam1_imu << std::endl;
        std::cout << "t_cam1_imu:\n" << t_cam1_imu << std::endl;

        printf("ImageProcessor end ===========================================\n");

        return true;
    }

    bool ImageProcessor::initialize() {
        if (!loadParameters())
            return false;
        std::cout << "Finish loading parameters..." << std::endl;
        detector_ptr = cv::FastFeatureDetector::create(processor_config.fast_threshold);

        debug_.open("debug_imageprocessor.txt");

        return true;
    }

    void ImageProcessor::stereoCallback(const mynt::Image &cam0_img, const mynt::Image &cam1_img) {
        // Get the current image.
        cam0_curr_img_ptr->time_stamp = cam0_img.time_stamp;
        cam1_curr_img_ptr->time_stamp = cam1_img.time_stamp;

        cam0_curr_img_ptr->image = cam0_img.image.clone();
        cam1_curr_img_ptr->image = cam1_img.image.clone();

        // Build the image pyramids once since they're used at multiple places
        createImagePyramids();

        // Detect features in the first frame.
        if (is_first_img) {
            initializeFirstFrame();

            is_first_img = false;
            // Draw results.
            drawFeaturesStereo();
        } else {
            // Track the feature in the previous image.
            trackFeatures();
            // Add new features into the current image.
            addNewFeatures();
            // Add new features into the current image.
            pruneGridFeatures();
            // Draw results.
            drawFeaturesStereo();
        }

        //updateFeatureLifetime();

        // Publish features in the current image.
        publish();

        // Update the previous image and previous features.
        cam0_prev_img_ptr = cam0_curr_img_ptr;
        prev_features_ptr = curr_features_ptr;
        std::swap(prev_cam0_pyramid_, curr_cam0_pyramid_);

        // Initialize the current features to empty vectors.
        curr_features_ptr.reset(new GridFeatures());
        for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code) {
            (*curr_features_ptr)[code] = std::vector<FeatureMetaData>(0);
        }

        return;
    }

    void ImageProcessor::imuCallback(const mynt::ImuConstPtr &msg) {
        // Wait for the first image to be set.
        if (is_first_img)
            return;
        imu_msg_buffer.push_back(*msg);
        return;
    }

    void ImageProcessor::createImagePyramids() {
        const cv::Mat &curr_cam0_img = cam0_curr_img_ptr->image;
        const cv::Mat &curr_cam1_img = cam1_curr_img_ptr->image;

//        cv::buildOpticalFlowPyramid(
//                curr_cam0_img, curr_cam0_pyramid_,
//                cv::Size(processor_config.patch_size, processor_config.patch_size),
//                processor_config.pyramid_levels, true, cv::BORDER_REFLECT_101,
//                cv::BORDER_CONSTANT, false);
//
//        cv::buildOpticalFlowPyramid(
//                curr_cam1_img, curr_cam1_pyramid_,
//                cv::Size(processor_config.patch_size, processor_config.patch_size),
//                processor_config.pyramid_levels, true, cv::BORDER_REFLECT_101,
//                cv::BORDER_CONSTANT, false);

        curr_cam0_pyramid_.clear();
        curr_cam1_pyramid_.clear();
        cv::Mat tmp1, tmp2;
#if 1
        for (int i = 0; i < 4; i++) { // the downsampling step of the Gaussian pyramid construction
            if(i == 0) {
                curr_cam0_pyramid_.push_back(curr_cam0_img);
                curr_cam1_pyramid_.push_back(curr_cam1_img);
                continue;
            }
            cv::Mat img1_last = curr_cam0_pyramid_[i-1];
            cv::pyrDown(img1_last, tmp1, img1_last.size() / 2);
            curr_cam0_pyramid_.push_back(tmp1);
            cv::Mat img2_last = curr_cam1_pyramid_[i-1];
            cv::pyrDown(img2_last, tmp2, img2_last.size() / 2);
            curr_cam1_pyramid_.push_back(tmp2);
        }
#else
        double scale = 1.0;
        for (int i = 0; i < 4; i++) {
            cv::resize(curr_cam0_img, tmp1, cv::Size(curr_cam0_img.cols * scale, curr_cam0_img.rows * scale));
            curr_cam0_pyramid_.push_back(tmp1);
            cv::resize(curr_cam1_img, tmp2, cv::Size(curr_cam1_img.cols * scale, curr_cam1_img.rows * scale));
            curr_cam1_pyramid_.push_back(tmp2);
            scale *= 0.5;
        }
#endif
    }

    void ImageProcessor::initializeFirstFrame() {
        // Size of each grid.
        const cv::Mat &img = cam0_curr_img_ptr->image;
        static int grid_height = img.rows / processor_config.grid_row;
        static int grid_width  = img.cols / processor_config.grid_col;

        // Detect new features on the frist image.
        std::vector<cv::KeyPoint> new_features(0);
        detector_ptr->detect(img, new_features);

        // Find the stereo matched points for the newly detected features.
        std::vector<mynt::Point2f> cam0_points(new_features.size());
        for (int i = 0; i < new_features.size(); ++i)
            cam0_points[i] = mynt::Point2f(new_features[i].pt.x, new_features[i].pt.y);

        std::vector<mynt::Point2f> cam1_points(0);
        std::vector<unsigned char> inlier_markers(0);
        stereoMatch(cam0_points, cam1_points, inlier_markers);

        std::vector<mynt::Point2f> cam0_inliers(0);
        std::vector<mynt::Point2f> cam1_inliers(0);
        std::vector<float> response_inliers(0);
        for (int i = 0; i < inlier_markers.size(); ++i) {
            if (inlier_markers[i] == 0)
                continue;
            cam0_inliers.push_back(cam0_points[i]);
            cam1_inliers.push_back(cam1_points[i]);
            response_inliers.push_back(new_features[i].response);
        }

        // Group the features into grids
        GridFeatures grid_new_features;
        for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code)
            grid_new_features[code] = std::vector<FeatureMetaData>(0);

        for (int i = 0; i < cam0_inliers.size(); ++i) {
            const mynt::Point2f &cam0_point = cam0_inliers[i];
            const mynt::Point2f &cam1_point = cam1_inliers[i];
            const float &response = response_inliers[i];

            int row = static_cast<int>(cam0_point.y / grid_height);
            int col = static_cast<int>(cam0_point.x / grid_width);
            int code = row * processor_config.grid_col + col;

            FeatureMetaData new_feature;
            new_feature.response = response;
            new_feature.cam0_point = cam0_point;
            new_feature.cam1_point = cam1_point;;
            grid_new_features[code].push_back(new_feature);
        }

        // Sort the new features in each grid based on its response.
        for (auto &item : grid_new_features)
            std::sort(item.second.begin(), item.second.end(), &ImageProcessor::featureCompareByResponse);

        // Collect new features within each grid with high response.
        for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code) {
            std::vector<FeatureMetaData> &features_this_grid = (*curr_features_ptr)[code];
            std::vector<FeatureMetaData> &new_features_this_grid = grid_new_features[code];

            for (int k = 0; k < processor_config.grid_min_feature_num && k < new_features_this_grid.size(); ++k) {
                features_this_grid.push_back(new_features_this_grid[k]);
                features_this_grid.back().id = next_feature_id++;
                features_this_grid.back().lifetime = 1;
            }
        }

        return;
    }

    void ImageProcessor::predictFeatureTracking(
            const std::vector<mynt::Point2f> &input_pts,
            const mynt::RotationMatrix &R_p_c,
            const mynt::Vector4 &intrinsics,
            std::vector<mynt::Point2f> &compensated_pts) {

        // Return directly if there are no input features.
        if (input_pts.size() == 0) {
            compensated_pts.clear();
            return;
        }
        compensated_pts.resize(input_pts.size());

        // Intrinsic matrix.
        mynt::FLOAT kval[9] = {
                intrinsics[0], 0.0, intrinsics[2],
                0.0, intrinsics[1], intrinsics[3],
                0.0, 0.0, 1.0};
        mynt::Matrix K(3, 3, kval);
        mynt::Matrix H = K * R_p_c * K.inv();

        for (int i = 0; i < input_pts.size(); ++i) {
            mynt::Vector3 p1({input_pts[i].x, input_pts[i].y, 1.0f});
            mynt::Vector3 p2 = H * p1;
            compensated_pts[i].x = p2[0] / p2[2];
            compensated_pts[i].y = p2[1] / p2[2];
        }

        return;
    }

    void ImageProcessor::trackFeatures() {
        // Size of each grid.
        static int grid_height = cam0_curr_img_ptr->image.rows / processor_config.grid_row;
        static int grid_width  = cam0_curr_img_ptr->image.cols / processor_config.grid_col;

        // Compute a rough relative rotation which takes a vector from the previous frame to the current frame.
        mynt::RotationMatrix cam0_R_p_c;
        mynt::RotationMatrix cam1_R_p_c;
        integrateImuData(cam0_R_p_c, cam1_R_p_c);

        // Organize the features in the previous image.
        std::vector<FeatureIDType> prev_ids(0);
        std::vector<int> prev_lifetime(0);
        std::vector<mynt::Point2f> prev_cam0_points(0);
        std::vector<mynt::Point2f> prev_cam1_points(0);

        for (const auto &item : *prev_features_ptr) {
            for (const auto &prev_feature : item.second) {
                prev_ids.push_back(prev_feature.id);
                prev_lifetime.push_back(prev_feature.lifetime);
                mynt::Point2f p_pt0(prev_feature.cam0_point.x, prev_feature.cam0_point.y);
                mynt::Point2f p_pt1(prev_feature.cam1_point.x, prev_feature.cam1_point.y);
                prev_cam0_points.push_back(p_pt0);
                prev_cam1_points.push_back(p_pt1);
            }
        }

        // Number of the features before tracking.
        before_tracking = prev_cam0_points.size();

        // Abort tracking if there is no features in the previous frame.
        if (prev_ids.size() == 0) return;

        // Track features using LK optical flow method.
        std::vector<mynt::Point2f> curr_cam0_points(0);
        std::vector<unsigned char> track_inliers(0);

        predictFeatureTracking(prev_cam0_points, cam0_R_p_c, cam0_intrinsics, curr_cam0_points);

        // TODO
        std::vector<cv::Point2f> cv_curr_cam0_points(curr_cam0_points.size());
        for(int i=0; i<curr_cam0_points.size(); ++i)
            cv_curr_cam0_points[i] = cv::Point2f(curr_cam0_points[i].x, curr_cam0_points[i].y);
        std::vector<cv::Point2f> cv_prev_cam0_points(prev_cam0_points.size());
        for(int i=0; i<prev_cam0_points.size(); ++i)
            cv_prev_cam0_points[i] = cv::Point2f(prev_cam0_points[i].x, prev_cam0_points[i].y);

//        cv::calcOpticalFlowPyrLK(
//                prev_cam0_pyramid_, curr_cam0_pyramid_,
//                cv_prev_cam0_points, cv_curr_cam0_points,
//                track_inliers, cv::noArray(),
//                cv::Size(processor_config.patch_size, processor_config.patch_size),
//                processor_config.pyramid_levels,
//                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
//                             processor_config.max_iteration,
//                             processor_config.track_precision),
//                cv::OPTFLOW_USE_INITIAL_FLOW);

        mynt::OpticalFlowMultiLevel(prev_cam0_pyramid_, curr_cam0_pyramid_, cv_prev_cam0_points, cv_curr_cam0_points, track_inliers, 15, 30);

        for(int i=0; i<cv_curr_cam0_points.size(); ++i)
            curr_cam0_points[i] = mynt::Point2f(cv_curr_cam0_points[i].x, cv_curr_cam0_points[i].y);

        // Mark those tracked points out of the image region as untracked.
        for (int i = 0; i < curr_cam0_points.size(); ++i) {
            if (track_inliers[i] == 0)
                continue;
            if (curr_cam0_points[i].y < 0 ||
                curr_cam0_points[i].y > cam0_curr_img_ptr->image.rows - 1 ||
                curr_cam0_points[i].x < 0 ||
                curr_cam0_points[i].x > cam0_curr_img_ptr->image.cols - 1)
                track_inliers[i] = 0;
        }

        // Collect the tracked points.
        std::vector<FeatureIDType> prev_tracked_ids(0);
        std::vector<int> prev_tracked_lifetime(0);
        std::vector<mynt::Point2f> prev_tracked_cam0_points(0);
        std::vector<mynt::Point2f> prev_tracked_cam1_points(0);
        std::vector<mynt::Point2f> curr_tracked_cam0_points(0);

        removeUnmarkedElements(prev_ids, track_inliers, prev_tracked_ids);
        removeUnmarkedElements(prev_lifetime, track_inliers, prev_tracked_lifetime);
        removeUnmarkedElements(prev_cam0_points, track_inliers, prev_tracked_cam0_points);
        removeUnmarkedElements(prev_cam1_points, track_inliers, prev_tracked_cam1_points);
        removeUnmarkedElements(curr_cam0_points, track_inliers, curr_tracked_cam0_points);

        // Number of features left after tracking.
        after_tracking = curr_tracked_cam0_points.size();

        // Outlier removal involves three steps, which forms a close
        // loop between the previous and current frames of cam0 (left)
        // and cam1 (right). Assuming the stereo matching between the
        // previous cam0 and cam1 images are correct, the three steps are:
        //
        // prev frames cam0 ----------> cam1
        //              |                |
        //              |ransac          |ransac
        //              |   stereo match |
        // curr frames cam0 ----------> cam1
        //
        // 1) Stereo matching between current images of cam0 and cam1.
        // 2) RANSAC between previous and current images of cam0.
        // 3) RANSAC between previous and current images of cam1.
        //
        // For Step 3, tracking between the images is no longer needed.
        // The stereo matching results are directly used in the RANSAC.

        // Step 1: stereo matching.
        std::vector<mynt::Point2f> curr_cam1_points(0);
        std::vector<unsigned char> match_inliers(0);
        stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);

        std::vector<FeatureIDType> prev_matched_ids(0);
        std::vector<int> prev_matched_lifetime(0);
        std::vector<mynt::Point2f> prev_matched_cam0_points(0);
        std::vector<mynt::Point2f> prev_matched_cam1_points(0);
        std::vector<mynt::Point2f> curr_matched_cam0_points(0);
        std::vector<mynt::Point2f> curr_matched_cam1_points(0);

        removeUnmarkedElements(prev_tracked_ids, match_inliers, prev_matched_ids);
        removeUnmarkedElements(prev_tracked_lifetime, match_inliers, prev_matched_lifetime);
        removeUnmarkedElements(prev_tracked_cam0_points, match_inliers, prev_matched_cam0_points);
        removeUnmarkedElements(prev_tracked_cam1_points, match_inliers, prev_matched_cam1_points);
        removeUnmarkedElements(curr_tracked_cam0_points, match_inliers, curr_matched_cam0_points);
        removeUnmarkedElements(curr_cam1_points, match_inliers, curr_matched_cam1_points);

        // Number of features left after stereo matching.
        after_matching = curr_matched_cam0_points.size();

//        // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1.
//        std::vector<int> cam0_ransac_inliers(0);
//        twoPointRansac(prev_matched_cam0_points, curr_matched_cam0_points,
//                       cam0_R_p_c, cam0_intrinsics, cam0_distortion_model,
//                       cam0_distortion_coeffs, processor_config.ransac_threshold,
//                       0.99, cam0_ransac_inliers);
//
//        std::vector<int> cam1_ransac_inliers(0);
//        twoPointRansac(prev_matched_cam1_points, curr_matched_cam1_points,
//                       cam1_R_p_c, cam1_intrinsics, cam1_distortion_model,
//                       cam1_distortion_coeffs, processor_config.ransac_threshold,
//                       0.99, cam1_ransac_inliers);

        // Number of features after ransac.
        after_ransac = 0;

        for (int i = 0; i < curr_matched_cam0_points.size(); ++i) {
//            if (cam0_ransac_inliers[i] == 0 || cam1_ransac_inliers[i] == 0)
//                continue;
            int row = static_cast<int>(curr_matched_cam0_points[i].y / grid_height);
            int col = static_cast<int>(curr_matched_cam0_points[i].x / grid_width);
            int code = row * processor_config.grid_col + col;
            (*curr_features_ptr)[code].push_back(FeatureMetaData());

            FeatureMetaData &grid_new_feature = (*curr_features_ptr)[code].back();
            grid_new_feature.id = prev_matched_ids[i];
            grid_new_feature.lifetime = ++prev_matched_lifetime[i];
            grid_new_feature.cam0_point = curr_matched_cam0_points[i];
            grid_new_feature.cam1_point = curr_matched_cam1_points[i];

            ++after_ransac;
        }

        // Compute the tracking rate.
        int prev_feature_num = 0;
        for (const auto &item : *prev_features_ptr)
            prev_feature_num += item.second.size();

        int curr_feature_num = 0;
        for (const auto &item : *curr_features_ptr)
            curr_feature_num += item.second.size();

//        printf(
//                "\033[0;32m candidates: %d; raw track: %d; stereo match: %d; ransac: %d/%d=%f\033[0m\n",
//                before_tracking, after_tracking, after_matching,
//                curr_feature_num, prev_feature_num,
//                static_cast<double>(curr_feature_num) /
//                (static_cast<double>(prev_feature_num) + 1e-5));

        return;
    }

    void ImageProcessor::stereoMatch(
            const std::vector<mynt::Point2f> &cam0_points,
            std::vector<mynt::Point2f> &cam1_points,
            std::vector<unsigned char> &inlier_markers) {

        if (cam0_points.size() == 0)
            return;

        if (cam1_points.size() == 0) {
            // Initialize cam1_points by projecting cam0_points to cam1 using the rotation from stereo extrinsics
            const mynt::RotationMatrix R_cam0_cam1 = R_cam1_imu.transpose() * R_cam0_imu;
            std::vector<mynt::Point2f> cam0_points_undistorted;
            undistortPoints(cam0_points, cam0_intrinsics, cam0_distortion_model, cam0_distortion_coeffs, cam0_points_undistorted, R_cam0_cam1);
            cam1_points = distortPoints(cam0_points_undistorted, cam1_intrinsics, cam1_distortion_model, cam1_distortion_coeffs);
        }

        // TODO
        std::vector<cv::Point2f> cv_cam0_points(cam0_points.size());
        for(int i=0; i<cam0_points.size(); ++i)
            cv_cam0_points[i] = cv::Point2f(cam0_points[i].x, cam0_points[i].y);
        std::vector<cv::Point2f> cv_cam1_points(cam1_points.size());
        for(int i=0; i<cam1_points.size(); ++i)
            cv_cam1_points[i] = cv::Point2f(cam1_points[i].x, cam1_points[i].y);

//        // Track features using LK optical flow method.
//        cv::calcOpticalFlowPyrLK(curr_cam0_pyramid_, curr_cam1_pyramid_,
//                                 cv_cam0_points, cv_cam1_points,
//                             inlier_markers, cv::noArray(),
//                             cv::Size(processor_config.patch_size, processor_config.patch_size),
//                             processor_config.pyramid_levels,
//                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
//                                          processor_config.max_iteration,
//                                          processor_config.track_precision),
//                             cv::OPTFLOW_USE_INITIAL_FLOW);

        mynt::OpticalFlowMultiLevel(curr_cam0_pyramid_, curr_cam1_pyramid_, cv_cam0_points, cv_cam1_points, inlier_markers, 15, 30);

        for(int i=0; i<cv_cam1_points.size(); ++i)
            cam1_points[i] = mynt::Point2f(cv_cam1_points[i].x, cv_cam1_points[i].y);

        // Mark those tracked points out of the image region as untracked.
        for (int i = 0; i < cam1_points.size(); ++i) {
            if (inlier_markers[i] == 0)
                continue;
            if (cam1_points[i].y < 0 ||
                cam1_points[i].y > cam1_curr_img_ptr->image.rows - 1 ||
                cam1_points[i].x < 0 ||
                cam1_points[i].x > cam1_curr_img_ptr->image.cols - 1)
                inlier_markers[i] = 0;
        }

        // Compute the relative rotation between the cam0
        // frame and cam1 frame.
        const mynt::RotationMatrix R_cam0_cam1 = R_cam1_imu.transpose() *  R_cam0_imu;
        const mynt::Vector3 t_cam0_cam1 = R_cam1_imu.transpose() * (t_cam0_imu - t_cam1_imu);
        // Compute the essential matrix.
        const mynt::Matrix t_cam0_cam1_hat = mynt::skew_symmetric(t_cam0_cam1);
        const mynt::Matrix E = t_cam0_cam1_hat * R_cam0_cam1;

        // Further remove outliers based on the known essential matrix.
        std::vector<mynt::Point2f> v_pts0;
        for(int i=0; i<cam0_points.size(); ++i)
            v_pts0.push_back(cam0_points[i]);
        std::vector<mynt::Point2f> v_pts1;
        for(int i=0; i<cam1_points.size(); ++i)
            v_pts1.push_back(cam1_points[i]);

        std::vector<mynt::Point2f> cam0_points_undistorted(0);
        std::vector<mynt::Point2f> cam1_points_undistorted(0);
        undistortPoints(v_pts0, cam0_intrinsics, cam0_distortion_model, cam0_distortion_coeffs, cam0_points_undistorted);
        undistortPoints(v_pts1, cam1_intrinsics, cam1_distortion_model, cam1_distortion_coeffs, cam1_points_undistorted);

        double norm_pixel_unit = 4.0 / (cam0_intrinsics[0] + cam0_intrinsics[1] + cam1_intrinsics[0] + cam1_intrinsics[1]);

        for (int i = 0; i < cam0_points_undistorted.size(); ++i) {
            if (inlier_markers[i] == 0)
                continue;
            mynt::Vector3 pt0({cam0_points_undistorted[i].x, cam0_points_undistorted[i].y, 1.0});
            mynt::Vector3 pt1({cam1_points_undistorted[i].x, cam1_points_undistorted[i].y, 1.0});
            mynt::Vector3 epipolar_line = E * pt0;
            double error = fabs(pt1.dot(epipolar_line)) / sqrt(epipolar_line[0] * epipolar_line[0] + epipolar_line[1] * epipolar_line[1]);
            if (error > processor_config.stereo_threshold * norm_pixel_unit)
                inlier_markers[i] = 0;
        }

        return;
    }

    void ImageProcessor::addNewFeatures() {
        const cv::Mat &curr_img = cam0_curr_img_ptr->image;

        // Size of each grid.
        static int grid_height = cam0_curr_img_ptr->image.rows / processor_config.grid_row;
        static int grid_width  = cam0_curr_img_ptr->image.cols / processor_config.grid_col;

        // Create a mask to avoid redetecting existing features.
        cv::Mat mask(curr_img.rows, curr_img.cols, CV_8U, cv::Scalar(1));

        for (const auto &features : *curr_features_ptr) {
            for (const auto &feature : features.second) {
                const int y = static_cast<int>(feature.cam0_point.y);
                const int x = static_cast<int>(feature.cam0_point.x);

                int up_lim = y - 2, bottom_lim = y + 3, left_lim = x - 2, right_lim = x + 3;
                if (up_lim < 0) up_lim = 0;
                if (bottom_lim > curr_img.rows) bottom_lim = curr_img.rows;
                if (left_lim < 0) left_lim = 0;
                if (right_lim > curr_img.cols) right_lim = curr_img.cols;

                cv::Range row_range(up_lim, bottom_lim);
                cv::Range col_range(left_lim, right_lim);
                mask(row_range, col_range) = 0;
            }
        }

        // Detect new features.
        std::vector<cv::KeyPoint> new_features(0);
        detector_ptr->detect(curr_img, new_features, mask);

        // Collect the new detected features based on the grid.
        // Select the ones with top response within each grid afterwards.
        std::vector<std::vector<cv::KeyPoint> > new_feature_sieve(processor_config.grid_row * processor_config.grid_col);
        for (const auto &feature : new_features) {
            int row = static_cast<int>(feature.pt.y / grid_height);
            int col = static_cast<int>(feature.pt.x / grid_width);
            new_feature_sieve[row * processor_config.grid_col + col].push_back(feature);
        }

        new_features.clear();
        for (auto &item : new_feature_sieve) {
            if (item.size() > processor_config.grid_max_feature_num) {
                std::sort(item.begin(), item.end(), &ImageProcessor::keyPointCompareByResponse);
                item.erase(item.begin() + processor_config.grid_max_feature_num, item.end());
            }
            new_features.insert(new_features.end(), item.begin(), item.end());
        }

        int detected_new_features = new_features.size();

        // Find the stereo matched points for the newly detected features.
        std::vector<mynt::Point2f> cam0_points(new_features.size());
        for (int i = 0; i < new_features.size(); ++i)
            cam0_points[i] = mynt::Point2f(new_features[i].pt.x, new_features[i].pt.y);

        std::vector<mynt::Point2f> cam1_points(0);
        std::vector<unsigned char> inlier_markers(0);
        stereoMatch(cam0_points, cam1_points, inlier_markers);

        std::vector<mynt::Point2f> cam0_inliers(0);
        std::vector<mynt::Point2f> cam1_inliers(0);
        std::vector<float> response_inliers(0);
        for (int i = 0; i < inlier_markers.size(); ++i) {
            if (inlier_markers[i] == 0)
                continue;
            cam0_inliers.push_back(cam0_points[i]);
            cam1_inliers.push_back(cam1_points[i]);
            response_inliers.push_back(new_features[i].response);
        }

        int matched_new_features = cam0_inliers.size();

        if (matched_new_features < 5 &&
            static_cast<double>(matched_new_features) /
            static_cast<double>(detected_new_features) < 0.1)
            printf("Images at [%f] seems unsynced...", cam0_curr_img_ptr->time_stamp);

        // Group the features into grids
        GridFeatures grid_new_features;
        for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code)
            grid_new_features[code] = std::vector<FeatureMetaData>(0);

        for (int i = 0; i < cam0_inliers.size(); ++i) {
            const mynt::Point2f &cam0_point = cam0_inliers[i];
            const mynt::Point2f &cam1_point = cam1_inliers[i];
            const float &response = response_inliers[i];

            int row = static_cast<int>(cam0_point.y / grid_height);
            int col = static_cast<int>(cam0_point.x / grid_width);
            int code = row * processor_config.grid_col + col;

            FeatureMetaData new_feature;
            new_feature.response = response;
            new_feature.cam0_point = cam0_point;
            new_feature.cam1_point = cam1_point;
            grid_new_features[code].push_back(new_feature);
        }

        // Sort the new features in each grid based on its response.
        for (auto &item : grid_new_features)
            std::sort(item.second.begin(), item.second.end(), &ImageProcessor::featureCompareByResponse);

        int new_added_feature_num = 0;
        // Collect new features within each grid with high response.
        for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code) {
            std::vector<FeatureMetaData> &features_this_grid = (*curr_features_ptr)[code];
            std::vector<FeatureMetaData> &new_features_this_grid = grid_new_features[code];

            if (features_this_grid.size() >= processor_config.grid_min_feature_num)
                continue;

            int vacancy_num = processor_config.grid_min_feature_num - features_this_grid.size();
            for (int k = 0; k < vacancy_num && k < new_features_this_grid.size(); ++k) {
                features_this_grid.push_back(new_features_this_grid[k]);
                features_this_grid.back().id = next_feature_id++;
                features_this_grid.back().lifetime = 1;

                ++new_added_feature_num;
            }
        }

//        printf("\033[0;33m detected: %d; matched: %d; new added feature: %d\033[0m\n",
//            detected_new_features, matched_new_features, new_added_feature_num);

        return;
    }

    void ImageProcessor::pruneGridFeatures() {
        for (auto &item : *curr_features_ptr) {
            auto &grid_features = item.second;
            // Continue if the number of features in this grid does not exceed the upper bound.
            if (grid_features.size() <= processor_config.grid_max_feature_num)
                continue;
            std::sort(grid_features.begin(), grid_features.end(), &ImageProcessor::featureCompareByLifetime);
            grid_features.erase(grid_features.begin() + processor_config.grid_max_feature_num, grid_features.end());
        }
        return;
    }

    void ImageProcessor::undistortPoints(
            const std::vector<mynt::Point2f> &pts_in,
            const mynt::Vector4 &intrinsics,
            const std::string &distortion_model,
            const mynt::Vector4 &distortion_coeffs,
            std::vector<mynt::Point2f> &pts_out,
            const mynt::Mat3 &rectification_matrix,
            const mynt::Vector4 &new_intrinsics) {

        if (pts_in.empty())
            return;

//        const cv::Matx33d cvK(
//                intrinsics[0], 0.0, intrinsics[2],
//                0.0, intrinsics[1], intrinsics[3],
//                0.0, 0.0, 1.0);
//
//        const cv::Matx33d cv_K_new(
//                new_intrinsics[0], 0.0, new_intrinsics[2],
//                0.0, new_intrinsics[1], new_intrinsics[3],
//                0.0, 0.0, 1.0);

        mynt::Mat3 K = mynt::Matrix::eye(3);
        K(0,0) = intrinsics[0];
        K(1,1) = intrinsics[1];
        K(0,2) = intrinsics[2];
        K(1,2) = intrinsics[3];

        mynt::Mat3 P = mynt::Matrix::eye(3);
        P(0,0) = new_intrinsics[0];
        P(1,1) = new_intrinsics[1];
        P(0,2) = new_intrinsics[2];
        P(1,2) = new_intrinsics[3];

        // TODO: undistort_points_fisheye

        std::vector<cv::Point2f> cv_pts_out;

        if (distortion_model == "radtan") {
//            cv::undistortPoints(cv_pts_in, cv_pts_out, cvK, vector4_to_cvvec4d(distortion_coeffs), rectification_matrix, cv_K_new);
            mynt::undistort_points(pts_in, pts_out, K, distortion_coeffs, rectification_matrix, P);
        } else if (distortion_model == "equidistant") {
//            cv::fisheye::undistortPoints(cv_pts_in, cv_pts_out, cvK, vector4_to_cvvec4d(distortion_coeffs), rectification_matrix, cv_K_new);
            mynt::undistort_points_fisheye(pts_in, pts_out, K, distortion_coeffs, rectification_matrix, P);
        } else {
            printf("The model %s is unrecognized, use radtan instead...", distortion_model.c_str());
//            cv::undistortPoints(cv_pts_in, cv_pts_out, cvK, vector4_to_cvvec4d(distortion_coeffs), rectification_matrix, cv_K_new);
            mynt::undistort_points(pts_in, pts_out, K, distortion_coeffs, rectification_matrix, P);
        }

        return;
    }

    std::vector<mynt::Point2f> ImageProcessor::distortPoints(
            const std::vector<mynt::Point2f> &pts_in,
            const mynt::Vector4 &intrinsics,
            const std::string &distortion_model,
            const mynt::Vector4 &distortion_coeffs) {
        mynt::Mat3 K = mynt::Matrix::eye(3);
        K(0,0) = intrinsics[0];
        K(1,1) = intrinsics[1];
        K(0,2) = intrinsics[2];
        K(1,2) = intrinsics[3];

        std::vector<mynt::Point2f> pts_out;

        if (distortion_model == "radtan") {
            mynt::project_points(pts_in, pts_out, mynt::Vector3(), mynt::Vector3(), K, distortion_coeffs);
        } else if (distortion_model == "equidistant") {
            // TODO
//            cv::fisheye::distortPoints(cv_pts_in, cv_pts_out, cvK, vector4_to_cvvec4d(distortion_coeffs));
            mynt::distort_points_fisheye(pts_in, pts_out, K, distortion_coeffs);
        } else {
            printf("The model %s is unrecognized, using radtan instead...", distortion_model.c_str());
            mynt::project_points(pts_in, pts_out, mynt::Vector3(), mynt::Vector3(), K, distortion_coeffs);
        }

        return pts_out;
    }

    void ImageProcessor::integrateImuData(mynt::RotationMatrix &cam0_R_p_c, mynt::RotationMatrix &cam1_R_p_c) {
        // Find the start and the end limit within the imu msg buffer.
        auto begin_iter = imu_msg_buffer.begin();
        while (begin_iter != imu_msg_buffer.end()) {
            if (begin_iter->time_stamp - cam0_prev_img_ptr->time_stamp < -0.01)
                ++begin_iter;
            else
                break;
        }

        auto end_iter = begin_iter;
        while (end_iter != imu_msg_buffer.end()) {
            if (end_iter->time_stamp - cam0_curr_img_ptr->time_stamp < 0.005)
                ++end_iter;
            else
                break;
        }

        // Compute the mean angular velocity in the IMU frame.
        mynt::Vector3 mean_ang_vel;
        for (auto iter = begin_iter; iter < end_iter; ++iter)
            mean_ang_vel += iter->angular_velocity;

        if (end_iter - begin_iter > 0)
            mean_ang_vel *= 1.0f / (end_iter - begin_iter);

        // Transform the mean angular velocity from the IMU frame to the cam0 and cam1 frames.
        mynt::Vector3 cam0_mean_ang_vel = R_cam0_imu.transpose() * mean_ang_vel;
        mynt::Vector3 cam1_mean_ang_vel = R_cam1_imu.transpose() * mean_ang_vel;

        // Compute the relative rotation.
        double dtime = cam0_curr_img_ptr->time_stamp - cam0_prev_img_ptr->time_stamp;
        cam0_R_p_c = mynt::rodrigues(cam0_mean_ang_vel * dtime).transpose();
        cam1_R_p_c = mynt::rodrigues(cam1_mean_ang_vel * dtime).transpose();

        // Delete the useless and used imu messages.
        imu_msg_buffer.erase(imu_msg_buffer.begin(), end_iter);

        return;
    }

    void ImageProcessor::rescalePoints(
            std::vector<mynt::Point2f> &pts1, std::vector<mynt::Point2f> &pts2, float &scaling_factor) {

        scaling_factor = 0.0f;

        for (int i = 0; i < pts1.size(); ++i) {
            scaling_factor += sqrt(pts1[i].dot(pts1[i]));
            scaling_factor += sqrt(pts2[i].dot(pts2[i]));
        }

        scaling_factor = (pts1.size() + pts2.size()) / scaling_factor * sqrt(2.0f);

        for (int i = 0; i < pts1.size(); ++i) {
            pts1[i] *= scaling_factor;
            pts2[i] *= scaling_factor;
        }

        return;
    }

    void ImageProcessor::twoPointRansac(
            const std::vector<mynt::Point2f> &pts1,
            const std::vector<mynt::Point2f> &pts2,
            const mynt::RotationMatrix &R_p_c,
            const mynt::Vector4 &intrinsics,
            const std::string &distortion_model,
            const mynt::Vector4 &distortion_coeffs,
            const double &inlier_error,
            const double &success_probability,
            std::vector<int> &inlier_markers) {

        // Check the size of input point size.
        if (pts1.size() != pts2.size())
            printf("Sets of different size (%lu and %lu) are used...", pts1.size(), pts2.size());

        double norm_pixel_unit = 2.0 / (intrinsics[0] + intrinsics[1]);
        int iter_num = static_cast<int>(std::ceil(log(1 - success_probability) / log(1 - 0.7 * 0.7)));

        // Initially, mark all points as inliers.
        inlier_markers.clear();
        inlier_markers.resize(pts1.size(), 1);

        // Undistort all the points.
        std::vector<mynt::Point2f> pts1_undistorted(pts1.size());
        std::vector<mynt::Point2f> pts2_undistorted(pts2.size());
        undistortPoints(pts1, intrinsics, distortion_model, distortion_coeffs, pts1_undistorted);
        undistortPoints(pts2, intrinsics, distortion_model, distortion_coeffs, pts2_undistorted);

        // Compenstate the points in the previous image with
        // the relative rotation.
        for (auto &pt : pts1_undistorted) {
            mynt::Vector3 pt_h({pt.x, pt.y, 1.0f});
            //Vec3f pt_hc = dR * pt_h;
            mynt::Vector3 pt_hc = R_p_c * pt_h;
            pt.x = pt_hc[0];
            pt.y = pt_hc[1];
        }

        // Normalize the points to gain numerical stability.
        float scaling_factor = 0.0f;
        rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
        norm_pixel_unit *= scaling_factor;

        // Compute the difference between previous and current points,
        // which will be used frequently later.
        std::vector<mynt::Point2f> pts_diff(pts1_undistorted.size());
        for (int i = 0; i < pts1_undistorted.size(); ++i)
            pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];

        // Mark the point pairs with large difference directly.
        // BTW, the mean distance of the rest of the point pairs
        // are computed.
        double mean_pt_distance = 0.0;
        int raw_inlier_cntr = 0;
        for (int i = 0; i < pts_diff.size(); ++i) {
            double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
            // 25 pixel distance is a pretty large tolerance for normal motion.
            // However, to be used with aggressive motion, this tolerance should
            // be increased significantly to match the usage.
            if (distance > 50.0 * norm_pixel_unit) {
                inlier_markers[i] = 0;
            } else {
                mean_pt_distance += distance;
                ++raw_inlier_cntr;
            }
        }
        mean_pt_distance /= raw_inlier_cntr;

        // If the current number of inliers is less than 3, just mark
        // all input as outliers. This case can happen with fast
        // rotation where very few features are tracked.
        if (raw_inlier_cntr < 3) {
            for (auto &marker : inlier_markers) marker = 0;
            return;
        }

        // Before doing 2-point RANSAC, we have to check if the motion
        // is degenerated, meaning that there is no translation between
        // the frames, in which case, the model of the RANSAC does not
        // work. If so, the distance between the matched points will
        // be almost 0.
        //if (mean_pt_distance < inlier_error*norm_pixel_unit) {
        if (mean_pt_distance < norm_pixel_unit) {
            //ROS_WARN_THROTTLE(1.0, "Degenerated motion...");
            for (int i = 0; i < pts_diff.size(); ++i) {
                if (inlier_markers[i] == 0) continue;
                if (sqrt(pts_diff[i].dot(pts_diff[i])) >
                    inlier_error * norm_pixel_unit)
                    inlier_markers[i] = 0;
            }
            return;
        }

        // In the case of general motion, the RANSAC model can be applied.
        // The three column corresponds to tx, ty, and tz respectively.
        mynt::Matrix coeff_t(pts_diff.size(), 3);
        for (int i = 0; i < pts_diff.size(); ++i) {
            coeff_t(i, 0) = pts_diff[i].y;
            coeff_t(i, 1) = -pts_diff[i].x;
            coeff_t(i, 2) = pts1_undistorted[i].x * pts2_undistorted[i].y -
                            pts1_undistorted[i].y * pts2_undistorted[i].x;
        }

        std::vector<int> raw_inlier_idx;
        for (int i = 0; i < inlier_markers.size(); ++i) {
            if (inlier_markers[i] != 0)
                raw_inlier_idx.push_back(i);
        }

        std::vector<int> best_inlier_set;
        double best_error = 1e10;

        for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx) {
            // Randomly select two point pairs.
            // Although this is a weird way of selecting two pairs, but it is able to efficiently avoid selecting repetitive pairs.
            int select_idx1     = mynt::uniform_integer(0, raw_inlier_idx.size() - 1);
            int select_idx_diff = mynt::uniform_integer(1, raw_inlier_idx.size() - 1);
            int select_idx2 = select_idx1 + select_idx_diff < raw_inlier_idx.size() ?
                              select_idx1 + select_idx_diff :
                              select_idx1 + select_idx_diff - raw_inlier_idx.size();

            int pair_idx1 = raw_inlier_idx[select_idx1];
            int pair_idx2 = raw_inlier_idx[select_idx2];

            // Construct the model;
            mynt::Vector2 coeff_tx({coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0)});
            mynt::Vector2 coeff_ty({coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1)});
            mynt::Vector2 coeff_tz({coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2)});
            std::vector<double> coeff_l1_norm(3);
            coeff_l1_norm[0] = coeff_tx.l1norm();
            coeff_l1_norm[1] = coeff_ty.l1norm();
            coeff_l1_norm[2] = coeff_tz.l1norm();
            int base_indicator = min_element(coeff_l1_norm.begin(), coeff_l1_norm.end()) - coeff_l1_norm.begin();

            mynt::Vector3 model;
            if (base_indicator == 0) {
                mynt::Matrix A = mynt::vec2mat(coeff_ty, coeff_tz);
                mynt::Vector2 solution = A.inv() * (-coeff_tx);
                model[0] = 1.0;
                model[1] = solution[0];
                model[2] = solution[1];
            } else if (base_indicator == 1) {
                mynt::Matrix A = mynt::vec2mat(coeff_tx, coeff_tz);
                mynt::Vector2 solution = A.inv() * (-coeff_ty);
                model[0] = solution[0];
                model[1] = 1.0;
                model[2] = solution[1];
            } else {
                mynt::Matrix A = mynt::vec2mat(coeff_tx, coeff_ty);
                mynt::Vector2 solution = A.inv() * (-coeff_tz);
                model[0] = solution[0];
                model[1] = solution[1];
                model[2] = 1.0;
            }

            // Find all the inliers among point pairs.
            mynt::VectorX error = coeff_t * model;

            std::vector<int> inlier_set;
            for (int i = 0; i < error.size(); ++i) {
                if (inlier_markers[i] == 0) continue;
                if (std::abs(error[i]) < inlier_error * norm_pixel_unit)
                    inlier_set.push_back(i);
            }

            // If the number of inliers is small, the current
            // model is probably wrong.
            if (inlier_set.size() < 0.2 * pts1_undistorted.size())
                continue;

            // Refit the model using all of the possible inliers.
            mynt::VectorX coeff_tx_better(inlier_set.size());
            mynt::VectorX coeff_ty_better(inlier_set.size());
            mynt::VectorX coeff_tz_better(inlier_set.size());
            for (int i = 0; i < inlier_set.size(); ++i) {
                coeff_tx_better[i] = coeff_t(inlier_set[i], 0);
                coeff_ty_better[i] = coeff_t(inlier_set[i], 1);
                coeff_tz_better[i] = coeff_t(inlier_set[i], 2);
            }

            mynt::Vector3 model_better;
            if (base_indicator == 0) {
                mynt::Matrix A = mynt::vec2mat(coeff_ty_better, coeff_tz_better);
                mynt::Vector2 solution = (A.transpose() * A).inv() * A.transpose() * (-coeff_tx_better);
                model_better[0] = 1.0;
                model_better[1] = solution[0];
                model_better[2] = solution[1];
            } else if (base_indicator == 1) {
                mynt::Matrix A = mynt::vec2mat(coeff_tx_better, coeff_tz_better);
                mynt::Vector2 solution = (A.transpose() * A).inv() * A.transpose() * (-coeff_ty_better);
                model_better[0] = solution[0];
                model_better[1] = 1.0;
                model_better[2] = solution[1];
            } else {
                mynt::Matrix A = mynt::vec2mat(coeff_tx_better, coeff_ty_better);
                mynt::Vector2 solution = (A.transpose() * A).inv() * A.transpose() * (-coeff_tz_better);
                model_better[0] = solution[0];
                model_better[1] = solution[1];
                model_better[2] = 1.0;
            }

            // Compute the error and upate the best model if possible.
            mynt::VectorX new_error = coeff_t * model_better;

            double this_error = 0.0;
            for (const auto &inlier_idx : inlier_set)
                this_error += std::abs(new_error[inlier_idx]);
            this_error /= inlier_set.size();

            if (inlier_set.size() > best_inlier_set.size()) {
                best_error = this_error;
                best_inlier_set = inlier_set;
            }
        }

        // Fill in the markers.
        inlier_markers.clear();
        inlier_markers.resize(pts1.size(), 0);
        for (const auto &inlier_idx : best_inlier_set)
            inlier_markers[inlier_idx] = 1;

        //printf("inlier ratio: %lu/%lu\n", best_inlier_set.size(), inlier_markers.size());

        return;
    }

    void ImageProcessor::publish() {
        feature_msg_ptr_->time_stamp = cam0_curr_img_ptr->time_stamp;

        std::vector<FeatureIDType> curr_ids(0);
        std::vector<mynt::Point2f> curr_cam0_points(0);
        std::vector<mynt::Point2f> curr_cam1_points(0);

        for (const auto &grid_features : (*curr_features_ptr)) {
            for (const auto &feature : grid_features.second) {
                curr_ids.push_back(feature.id);
                curr_cam0_points.push_back(mynt::Point2f(feature.cam0_point.x, feature.cam0_point.y));
                curr_cam1_points.push_back(mynt::Point2f(feature.cam1_point.x, feature.cam1_point.y));
            }
        }

        std::vector<mynt::Point2f> curr_cam0_points_undistorted(0);
        std::vector<mynt::Point2f> curr_cam1_points_undistorted(0);
        undistortPoints(curr_cam0_points, cam0_intrinsics, cam0_distortion_model, cam0_distortion_coeffs, curr_cam0_points_undistorted);
        undistortPoints(curr_cam1_points, cam1_intrinsics, cam1_distortion_model, cam1_distortion_coeffs, curr_cam1_points_undistorted);

//        debug_ << std::fixed << std::setprecision(9) << feature_msg_ptr_->time_stamp
//               << " ================================= " << curr_ids.size() << std::endl;

        for (int i = 0; i < curr_ids.size(); ++i) {
            feature_msg_ptr_->features.push_back(FeatureMeasurement());
            feature_msg_ptr_->features[i].id = curr_ids[i];
            feature_msg_ptr_->features[i].u0 = curr_cam0_points_undistorted[i].x;
            feature_msg_ptr_->features[i].v0 = curr_cam0_points_undistorted[i].y;
            feature_msg_ptr_->features[i].u1 = curr_cam1_points_undistorted[i].x;
            feature_msg_ptr_->features[i].v1 = curr_cam1_points_undistorted[i].y;

//            FeatureMeasurement fm = feature_msg_ptr_->features[i];
//            debug_ << std::fixed << std::setprecision(5) << fm.u0 << ", " << fm.v0 << ", " << fm.u1 << ", " << fm.v1 << std::endl;
        }

//        debug_ << std::endl;

        // Publish tracking info.
        boost::shared_ptr<TrackingInfo> tracking_info_msg_ptr(new TrackingInfo());
        tracking_info_msg_ptr->time_stamp = cam0_curr_img_ptr->time_stamp;
        tracking_info_msg_ptr->before_tracking = before_tracking;
        tracking_info_msg_ptr->after_tracking = after_tracking;
        tracking_info_msg_ptr->after_matching = after_matching;
        tracking_info_msg_ptr->after_ransac = after_ransac;

        debug_ << std::fixed << std::setprecision(9)
               << tracking_info_msg_ptr->time_stamp << ": "
               << tracking_info_msg_ptr->before_tracking << ", "
               << tracking_info_msg_ptr->after_tracking << ", "
               << tracking_info_msg_ptr->after_matching << ", "
               << tracking_info_msg_ptr->after_ransac << std::endl;

        return;
    }

    void ImageProcessor::drawFeaturesStereo() {
        if (true) {
            // Colors for different features.
            cv::Scalar tracked(0, 255, 0);
            cv::Scalar new_feature(0, 0, 255);

            static int grid_height = cam0_curr_img_ptr->image.rows / processor_config.grid_row;
            static int grid_width  = cam0_curr_img_ptr->image.cols / processor_config.grid_col;

            // Create an output image.
            int img_height = cam0_curr_img_ptr->image.rows;
            int img_width = cam0_curr_img_ptr->image.cols;
            cv::Mat out_img(img_height, img_width * 2, CV_8UC3);
            cvtColor(cam0_curr_img_ptr->image, out_img.colRange(0, img_width), CV_GRAY2RGB);
            cvtColor(cam1_curr_img_ptr->image, out_img.colRange(img_width, img_width * 2), CV_GRAY2RGB);

            // Draw grids on the image.
            for (int i = 1; i < processor_config.grid_row; ++i) {
                cv::Point pt1(0, i * grid_height);
                cv::Point pt2(img_width * 2, i * grid_height);
                line(out_img, pt1, pt2, cv::Scalar(255, 0, 0));
            }
            for (int i = 1; i < processor_config.grid_col; ++i) {
                cv::Point pt1(i * grid_width, 0);
                cv::Point pt2(i * grid_width, img_height);
                line(out_img, pt1, pt2, cv::Scalar(255, 0, 0));
            }
            for (int i = 1; i < processor_config.grid_col; ++i) {
                cv::Point pt1(i * grid_width + img_width, 0);
                cv::Point pt2(i * grid_width + img_width, img_height);
                line(out_img, pt1, pt2, cv::Scalar(255, 0, 0));
            }

            // Collect features ids in the previous frame.
            std::vector<FeatureIDType> prev_ids(0);
            for (const auto &grid_features : *prev_features_ptr)
                for (const auto &feature : grid_features.second)
                    prev_ids.push_back(feature.id);

            // Collect feature points in the previous frame.
            std::map<FeatureIDType, cv::Point2f> prev_cam0_points;
            std::map<FeatureIDType, cv::Point2f> prev_cam1_points;
            for (const auto &grid_features : *prev_features_ptr)
                for (const auto &feature : grid_features.second) {
                    prev_cam0_points[feature.id] = cv::Point2f(feature.cam0_point.x, feature.cam0_point.y);
                    prev_cam1_points[feature.id] = cv::Point2f(feature.cam1_point.x, feature.cam1_point.y);
                }

            // Collect feature points in the current frame.
            std::map<FeatureIDType, cv::Point2f> curr_cam0_points;
            std::map<FeatureIDType, cv::Point2f> curr_cam1_points;
            for (const auto &grid_features : *curr_features_ptr)
                for (const auto &feature : grid_features.second) {
                    curr_cam0_points[feature.id] = cv::Point2f(feature.cam0_point.x, feature.cam0_point.y);
                    curr_cam1_points[feature.id] = cv::Point2f(feature.cam1_point.x, feature.cam1_point.y);
                }

            // Draw tracked features.
            for (const auto &id : prev_ids) {
                if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
                    curr_cam0_points.find(id) != curr_cam0_points.end()) {
                    cv::Point2f prev_pt0 = prev_cam0_points[id];
                    cv::Point2f prev_pt1 = prev_cam1_points[id] + cv::Point2f(img_width, 0.0);
                    cv::Point2f curr_pt0 = curr_cam0_points[id];
                    cv::Point2f curr_pt1 = curr_cam1_points[id] + cv::Point2f(img_width, 0.0);

                    cv::circle(out_img, curr_pt0, 3, tracked, -1);
                    cv::circle(out_img, curr_pt1, 3, tracked, -1);
                    cv::line(out_img, prev_pt0, curr_pt0, tracked, 1);
                    cv::line(out_img, prev_pt1, curr_pt1, tracked, 1);

                    prev_cam0_points.erase(id);
                    prev_cam1_points.erase(id);
                    curr_cam0_points.erase(id);
                    curr_cam1_points.erase(id);
                }
            }

            // Draw new features.
            for (const auto &new_cam0_point : curr_cam0_points) {
                cv::Point2f pt0 = new_cam0_point.second;
                cv::Point2f pt1 = curr_cam1_points[new_cam0_point.first] + cv::Point2f(img_width, 0.0);

                cv::circle(out_img, pt0, 3, new_feature, -1);
                cv::circle(out_img, pt1, 3, new_feature, -1);
            }

            cv::resize(out_img, out_img, cv::Size(out_img.cols*0.5, out_img.rows*0.5));
            cv::namedWindow("Feature");
            cv::imshow("Feature", out_img);
            cv::waitKey(5);
        }

        return;
    }

    void ImageProcessor::updateFeatureLifetime() {
        for (int code = 0; code < processor_config.grid_row * processor_config.grid_col; ++code) {
            std::vector<FeatureMetaData> &features = (*curr_features_ptr)[code];
            for (const auto &feature : features) {
                if (feature_lifetime.find(feature.id) == feature_lifetime.end())
                    feature_lifetime[feature.id] = 1;
                else
                    ++feature_lifetime[feature.id];
            }
        }

        return;
    }

    void ImageProcessor::featureLifetimeStatistics() {
        std::map<int, int> lifetime_statistics;
        for (const auto &data : feature_lifetime) {
            if (lifetime_statistics.find(data.second) == lifetime_statistics.end())
                lifetime_statistics[data.second] = 1;
            else
                ++lifetime_statistics[data.second];
        }

        for (const auto &data : lifetime_statistics)
            std::cout << data.first << " : " << data.second << std::endl;

        return;
    }

} // end namespace msckf_vio
