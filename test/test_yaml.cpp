#include "common/config_io.h"

#include <gtest/gtest.h>

TEST(ConfigIO, basis) {
    YAML::Node cfg_file = YAML::LoadFile("../config/camchain-imucam-euroc.yaml");

    mynt::Mat4 m4 = cfg_file["cam0"]["T_cam_imu"].as<mynt::Mat4>();
    mynt::Vector4 v4 = cfg_file["cam0"]["distortion_coeffs"].as<mynt::Vector4>();
    std::string cam_model = cfg_file["cam0"]["distortion_model"].as<std::string>();
    mynt::Vector2 v2 = cfg_file["cam0"]["resolution"].as<mynt::Vector2>();

    std::cout << "cam0 T_cam_imu: \n" << m4 << std::endl;
    std::cout << "cam0 distortion_coeffs: " << v4.transpose() << std::endl;
    std::cout << "cam0 distortion_model: " << cam_model << std::endl;
    std::cout << "cam0 resolution: " << v2.transpose() << std::endl;
}

