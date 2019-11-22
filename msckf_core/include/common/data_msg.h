//
// Created by cg on 8/20/19.
//

#ifndef MSCKF_VIO_MYNT_DATA_MSG_H
#define MSCKF_VIO_MYNT_DATA_MSG_H

//#include <boost/shared_ptr.hpp>

#include "maths/vector.h"
#include "cv/yimg.h"

namespace mynt {

    struct Image {
        double time_stamp;
        mynt::YImg8 image;
    };
    typedef std::shared_ptr<Image> ImagePtr;
    typedef std::shared_ptr<const Image> ImageConstPtr;

    struct Imu {
        double time_stamp;
        mynt::Vector3 angular_velocity;
        mynt::Vector3 linear_acceleration;
    };
    typedef std::shared_ptr<Imu> ImuPtr;
    typedef std::shared_ptr<const Imu> ImuConstPtr;

    struct FeatureMeasurement {
        unsigned int id;
        // Normalized feature coordinates (with identity intrinsic matrix)
        double u0; // horizontal coordinate in cam0
        double v0; // vertical coordinate in cam0
        double u1; // horizontal coordinate in cam0
        double v1; // vertical coordinate in cam0
    };
    typedef std::shared_ptr<FeatureMeasurement> FeatureMeasurementPtr;
    typedef std::shared_ptr<const FeatureMeasurement> FeatureMeasurementConstPtr;

    struct CameraMeasurement {
        double time_stamp;
        std::vector<FeatureMeasurement> features;
    };
    typedef std::shared_ptr<CameraMeasurement> CameraMeasurementPtr;
    typedef std::shared_ptr<const CameraMeasurement> CameraMeasurementConstPtr;

    struct TrackingInfo {
        double time_stamp;
        // Number of features after each outlier removal step.
        int before_tracking;
        int after_tracking;
        int after_matching;
        int after_ransac;
    };
}

#endif //MSCKF_VIO_MYNT_DATA_MSG_H
