#include "config_io.h"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>


TEST(ConfigIO, basis) {
    YAML::Node cfg_file = YAML::LoadFile("../config/camchain-imucam-euroc.yaml");

    Eigen::Matrix4d m4 = cfg_file["cam0"]["T_cam_imu"].as<Eigen::Matrix4d>();
    Eigen::Vector4d v4 = cfg_file["cam0"]["distortion_coeffs"].as<Eigen::Vector4d>();
    std::string cam_model = cfg_file["cam0"]["distortion_model"].as<std::string>();
    Eigen::Vector2d v2 = cfg_file["cam0"]["resolution"].as<Eigen::Vector2d>();

    std::cout << "cam0 T_cam_imu: \n" << m4.transpose() << std::endl;
    std::cout << "cam0 distortion_coeffs: " << v4.transpose() << std::endl;
    std::cout << "cam0 distortion_model: " << cam_model << std::endl;
    std::cout << "cam0 resolution: " << v2.transpose() << std::endl;

    cv::Mat mat(4,4,CV_64FC1);
    memcpy(mat.data, m4.data(), sizeof(double)*16);
    std::cout << "cv mat: \n" << mat << std::endl;
}

