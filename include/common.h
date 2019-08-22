//
// Created by cg on 8/20/19.
//

#ifndef MSCKF_VIO_MYNT_COMMON_H
#define MSCKF_VIO_MYNT_COMMON_H

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace mynt {

    struct Image {
        double time_stamp;
        cv::Mat image;
    };
    typedef boost::shared_ptr<Image> ImagePtr;
    typedef boost::shared_ptr<const Image> ImageConstPtr;

    struct Imu {
        double time_stamp;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
    };
    typedef boost::shared_ptr<Imu> ImuPtr;
    typedef boost::shared_ptr<const Imu> ImuConstPtr;

    struct FeatureMeasurement {
        unsigned int id;
        // Normalized feature coordinates (with identity intrinsic matrix)
        double u0; // horizontal coordinate in cam0
        double v0; // vertical coordinate in cam0
        double u1; // horizontal coordinate in cam0
        double v1; // vertical coordinate in cam0
    };
    typedef boost::shared_ptr<FeatureMeasurement> FeatureMeasurementPtr;
    typedef boost::shared_ptr<const FeatureMeasurement> FeatureMeasurementConstPtr;

    struct CameraMeasurement {
        double time_stamp;
        std::vector<FeatureMeasurement> features;
    };
    typedef boost::shared_ptr<CameraMeasurement> CameraMeasurementPtr;
    typedef boost::shared_ptr<const CameraMeasurement> CameraMeasurementConstPtr;

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
