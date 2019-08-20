#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

// params
std::string euroc_dir = "/home/cg/projects/datasets/MH_01_easy/mav0/";
int num_cams = 2;

std::vector<std::vector<std::pair<double,std::string>>> data_img(num_cams);

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
                double stamp_ns = std::stof(s);
                std::getline(stream, s, ',');
                std::string imgname = s.substr(0, s.size()-1);
                data_img[n].push_back(std::make_pair(stamp_ns, imgname));
            }
            cam_file.close();
        }
        is_init = true;
    }
    for (int i = 0; i < data_img[0].size(); ++i) {
        cv::Mat imgs[num_cams];
        for(int j=0; j<num_cams; ++j) {
            std::string img_path = euroc_dir + "/cam" + std::to_string(j) + "/data/" + data_img[j][i].second;
            imgs[j] = cv::imread(img_path, 0);
            if (imgs[j].empty()) {
                std::cerr << "ERROR: img is empty !!!" << std::endl;
                continue;
            }
            cv::imshow("cam"+std::to_string(j), imgs[j]);
            cv::waitKey(20);
        }
        usleep(30000);
    }
}

void process_imu_data() {
    std::vector<std::pair<double,std::pair<Eigen::Vector3d,Eigen::Vector3d>>> data_imu;
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
        double stamp_ns = std::stof(s);
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
        std::cout << "imu: " << gyr.transpose() << ", " << gyr.transpose() << std::endl;
        data_imu.push_back(std::make_pair(stamp_ns, std::make_pair(gyr,acc)));
        usleep(5000*2);
    }
    imu_file.close();
}

int main() {
    std::cout << "start run_euroc..." << std::endl;

    std::thread thd_image_data(process_image_data);
    std::thread thd_imu_data(process_imu_data);

    thd_image_data.join();
    thd_imu_data.join();

    return 0;
}