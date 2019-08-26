#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

#include "system.h"

// params
std::string euroc_dir = "/home/cg/projects/datasets/V1_01_easy/mav0/";
int num_cams = 2;

std::vector<std::vector<std::pair<double,std::string>>> data_img(num_cams);

mynt::SystemPtr system_ptr;

void process_image_data() {
    static bool is_init = false;
    if(!is_init) {
        for (int n = 0; n < num_cams; ++n) {
            std::ifstream cam_file(euroc_dir + "/cam" + std::to_string(n) + "/data.csv");
            if (!cam_file.good()) {
                std::cerr << "ERROR: no cam file found !!!" << std::endl;
                return;
            }
            std::string line;
            std::getline(cam_file, line);
            while (std::getline(cam_file, line)) {
                std::stringstream stream(line);
                std::string s;
                std::getline(stream, s, ',');
                std::string nanoseconds = s.substr(s.size() - 9, 9);
                std::string seconds = s.substr(0, s.size() - 9);
                double stamp_ns = std::stoi(seconds) * 1e9 + std::stoi(nanoseconds);
                std::getline(stream, s, ',');
                std::string imgname = s.substr(0, s.size()-1);
                data_img[n].push_back(std::make_pair(stamp_ns, imgname));
            }
            cam_file.close();
        }
        is_init = true;
    }
    int i = 0;
    while(i < data_img[0].size()) {
        mynt::Image imgs[num_cams];
        for(int j=0; j<num_cams; ++j) {
            imgs[j].time_stamp = data_img[j][i].first*1e-9; // to seconds;
            std::string img_path = euroc_dir + "/cam" + std::to_string(j) + "/data/" + data_img[j][i].second;
            imgs[j].image = cv::imread(img_path, 0);
            if (imgs[j].image.empty()) {
                std::cerr << "ERROR: img is empty !!!" << std::endl;
                continue;
            }
            // cv::imshow("cam"+std::to_string(j), imgs[j].image);
            // cv::waitKey(20);
        }

        system_ptr->stereo_callback(imgs[0], imgs[1]);

        usleep(50000*2);

        ++i;
    }
}

void process_imu_data() {
    std::ifstream imu_file(euroc_dir + "/imu0/data.csv");
    if (!imu_file.good()) {
        std::cerr << "ERROR: no imu file found !!!" << std::endl;
        return;
    }
    std::string line;
    std::getline(imu_file, line);
    while(std::getline(imu_file, line)) {
        std::stringstream stream(line);
        std::string s;
        std::getline(stream, s, ',');
        std::string nanoseconds = s.substr(s.size() - 9, 9);
        std::string seconds = s.substr(0, s.size() - 9);
        double stamp_ns = std::stoi(seconds) * 1e9 + std::stoi(nanoseconds);
        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            gyr[j] = std::stof(s);
        }
        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
            std::getline(stream, s, ',');
            acc[j] = std::stof(s);
        }
        // std::cout << "imu: " << gyr.transpose() << ", " << acc.transpose() << std::endl;

        boost::shared_ptr<mynt::Imu> imu(new mynt::Imu);
        imu->time_stamp = stamp_ns*1e-9; // to seconds
        imu->angular_velocity = gyr;
        imu->linear_acceleration = acc;

        system_ptr->imu_callback(imu);

        usleep(5000*2);
    }
    imu_file.close();
}

int main() {
    std::cout << "start run_euroc..." << std::endl;

    std::string file_cam_imu = "../config/camchain-imucam-euroc.yaml";

    system_ptr.reset(new mynt::System(file_cam_imu));

    std::thread thd_backend(&mynt::System::backend_callback, system_ptr);
    std::thread thd_image_data(process_image_data);
    std::thread thd_imu_data(process_imu_data);
    std::thread thd_draw(&mynt::System::draw, system_ptr);

    thd_image_data.join();
    thd_imu_data.join();
    thd_backend.join();
    thd_draw.join();

    return 0;
}