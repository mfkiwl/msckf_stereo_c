#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

#include <pangolin/pangolin.h>
using namespace pangolin;

#include "system.h"

mynt::SystemPtr system_ptr;

int main() {
    std::cout << "start run_euroc..." << std::endl;

    std::string file_cam_imu = "../config/camchain-imucam-euroc.yaml";
    std::string euroc_dir = "/home/cg/projects/datasets/V1_01_easy/mav0/";
    int num_cams = 2;
    double timestamp_first = 0.0;

    system_ptr.reset(new mynt::System(file_cam_imu));

    // init viewer
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 640, 480);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // image data
    std::vector<std::vector<std::pair<double,std::string>>> data_img(num_cams);
    for (int n = 0; n < num_cams; ++n) {
        std::ifstream cam_file(euroc_dir + "/cam" + std::to_string(n) + "/data.csv");
        if (!cam_file.good()) {
            std::cerr << "ERROR: no cam file found !!!" << std::endl;
            return -1;
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
    timestamp_first = data_img[0][0].first*1e-9; // to seconds;

    if(num_cams == 2)
        assert(data_img[0].size() == data_img[1].size());

    // imu data
    std::ifstream imu_file(euroc_dir + "/imu0/data.csv");
    if (!imu_file.good()) {
        std::cerr << "ERROR: no imu file found !!!" << std::endl;
        return -1;
    }
    std::string line;
    std::getline(imu_file, line);

    unsigned int imgs_size = data_img[0].size();
    int idx_img = 0;
    while(idx_img < imgs_size) {
        mynt::Image imgs[num_cams];
        for(int j=0; j<num_cams; ++j) {
            imgs[j].time_stamp = data_img[j][idx_img].first*1e-9; // to seconds;
            std::string img_path = euroc_dir + "/cam" + std::to_string(j) + "/data/" + data_img[j][idx_img].second;
            imgs[j].image = cv::imread(img_path, 0);
            if (imgs[j].image.empty()) {
                std::cerr << "ERROR: img is empty !!!" << std::endl;
                continue;
            }
//             cv::imshow("cam"+std::to_string(j), imgs[j].image);
//             cv::waitKey(20);
        }

        double t_img = imgs[0].time_stamp;

        double t_imu = 0.0;
        do {
            std::getline(imu_file, line);
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

            t_imu = imu->time_stamp;

        } while(t_imu <= t_img);

        system_ptr->stereo_callback(imgs[0], imgs[1]);

        system_ptr->backend_callback();

        // view
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(s_cam);
            glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
            glColor3f(0, 0, 1);
            pangolin::glDrawAxis(3);

            // draw poses
            glColor3f(0, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            // std::vector<Eigen::Vector3d> path_to_draw = path_to_draw_;
            int nPath_size = system_ptr->path_to_draw_.size();
            for (int i = 0; i < nPath_size - 1; ++i) {
                glVertex3f(system_ptr->path_to_draw_[i].x(), system_ptr->path_to_draw_[i].y(), system_ptr->path_to_draw_[i].z());
                glVertex3f(system_ptr->path_to_draw_[i + 1].x(), system_ptr->path_to_draw_[i + 1].y(), system_ptr->path_to_draw_[i + 1].z());
            }
            glEnd();

            pangolin::FinishFrame();
        }

        idx_img++;
    }

    imu_file.close();

    pangolin::QuitAll();

    std::cout << "end run_euroc..." << std::endl;

    return 0;
}
