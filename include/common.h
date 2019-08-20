//
// Created by cg on 8/20/19.
//

#ifndef MSCKF_VIO_MYNT_COMMON_H
#define MSCKF_VIO_MYNT_COMMON_H

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace mynt {

    struct Image {
        double time_stamp;
        cv::Mat image;
    };

    struct Imu {
        double time_stamp;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
    };

    struct FeatureMeasurement {
        unsigned int id;
        // Normalized feature coordinates (with identity intrinsic matrix)
        double u0; // horizontal coordinate in cam0
        double v0; // vertical coordinate in cam0
        double u1; // horizontal coordinate in cam0
        double v1; // vertical coordinate in cam0
    };

    struct CameraMeasurement {
        double time_stamp;
        std::vector<FeatureMeasurement> features;
    };

    struct TrackingInfo {
        double time_stamp;
        // Number of features after each outlier removal step.
        int before_tracking;
        int after_tracking;
        int after_matching;
        int after_ransac;
    };
}

#endif //MSCKF_VIO_MYNT_COMMON_H
