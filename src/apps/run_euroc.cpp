#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

#include <pangolin/pangolin.h>
using namespace pangolin;

#include "system.h"

// params
std::string euroc_dir = "/home/cg/projects/datasets/V1_01_easy/mav0/";
int num_cams = 2;
double timestamp_first = 0.0;

std::vector<std::vector<std::pair<double,std::string>>> data_img(num_cams);

mynt::SystemPtr system_ptr;

std::mutex mt_feature;

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
        timestamp_first = data_img[0][0].first*1e-9; // to seconds;
    }
    if(num_cams == 2)
        assert(data_img[0].size() == data_img[1].size());

    unsigned int imgs_size = data_img[0].size();
    for(int i = 0; i < imgs_size; ++i) {
        auto start = std::chrono::system_clock::now();

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

        mt_feature.lock();
        system_ptr->stereo_callback(imgs[0], imgs[1]); // ~25ms
        mt_feature.unlock();

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;

        std::this_thread::sleep_for(std::chrono::milliseconds(int((0.05-elapsed_seconds.count())*1e3)));
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
        auto start = std::chrono::system_clock::now();

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

        if(imu->time_stamp < timestamp_first) {
            continue;
        }

        system_ptr->imu_callback(imu);

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;

        std::this_thread::sleep_for(std::chrono::milliseconds(int((0.005-elapsed_seconds.count())*1e3)));
    }
    imu_file.close();
}

void process_backend() {
    while(true) {
        auto start = std::chrono::system_clock::now();
        mt_feature.lock();
        system_ptr->backend_callback();
        mt_feature.unlock();
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::this_thread::sleep_for(std::chrono::milliseconds(int((0.05 - elapsed_seconds.count()) * 1e3)));
    }
}

void draw() {
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

    while (pangolin::ShouldQuit() == false) {
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
        for(int i = 0; i < nPath_size-1; ++i)
        {
            glVertex3f(system_ptr->path_to_draw_[i].x(), system_ptr->path_to_draw_[i].y(), system_ptr->path_to_draw_[i].z());
            glVertex3f(system_ptr->path_to_draw_[i+1].x(), system_ptr->path_to_draw_[i+1].y(), system_ptr->path_to_draw_[i+1].z());
        }
        glEnd();

//            // points
//            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
//            {
//                glPointSize(5);
//                glBegin(GL_POINTS);
//                for(int i = 0; i < WINDOW_SIZE+1;++i)
//                {
//                    Vector3d p_wi = estimator.Ps[i];
//                    glColor3f(1, 0, 0);
//                    glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
//                }
//                glEnd();
//            }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

int main() {
    std::cout << "start run_euroc..." << std::endl;

    std::string file_cam_imu = "../config/camchain-imucam-euroc.yaml";

    system_ptr.reset(new mynt::System(file_cam_imu));

    std::thread thd_backend(process_backend);
    std::thread thd_image_data(process_image_data);
    std::thread thd_imu_data(process_imu_data);
    //std::thread thd_draw(&mynt::System::draw, system_ptr);
    std::thread thd_draw(draw);

    thd_image_data.join();
    thd_imu_data.join();
    thd_backend.join();
    thd_draw.join();

    pangolin::QuitAll();

    return 0;
}