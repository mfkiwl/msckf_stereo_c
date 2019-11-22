#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define ONLY_GL 0

#if ONLY_GL
#else
#include <pangolin/pangolin.h>
using namespace pangolin;
#endif

#include "system.h"

#include "glwindow/scenewindow.hpp"

mynt::SystemPtr system_ptr;

typedef unsigned long long int FeatureIDType;

void draw_features_stereo(
        mynt::YImg8 img_cam0, mynt::YImg8 img_cam1,
        std::vector<FeatureIDType> prev_ids,
        std::map<FeatureIDType, mynt::Point2f> prev_cam0_points,
        std::map<FeatureIDType, mynt::Point2f> prev_cam1_points,
        std::map<FeatureIDType, mynt::Point2f> curr_cam0_points,
        std::map<FeatureIDType, mynt::Point2f> curr_cam1_points,
        int grid_row, int grid_col
) {

    int img_height = img_cam0.rows();
    int img_width  = img_cam0.cols();

    static int grid_height = img_height / grid_row;
    static int grid_width  = img_width  / grid_col;

    // Create an output image.
    cv::Mat mat_01(img_height, img_width, CV_8UC1);
    cv::Mat mat_02(img_height, img_width, CV_8UC1);

    mempcpy(mat_01.data, img_cam0.data(), sizeof(unsigned char) * img_cam0.size().area());
    mempcpy(mat_02.data, img_cam1.data(), sizeof(unsigned char) * img_cam1.size().area());

    cv::Mat out_img(img_height, img_width * 2, CV_8UC3);
    cv::cvtColor(mat_01, out_img.colRange(0, img_width), CV_GRAY2RGB);
    cv::cvtColor(mat_02, out_img.colRange(img_width, img_width * 2), CV_GRAY2RGB);

    // Draw grids on the image.
    for (int i = 1; i < grid_row; ++i) {
        cv::Point pt1(0, i * grid_height);
        cv::Point pt2(img_width * 2, i * grid_height);
        cv::line(out_img, pt1, pt2, cv::Scalar(255, 0, 0));
    }
    for (int i = 1; i < grid_col; ++i) {
        cv::Point pt1(i * grid_width, 0);
        cv::Point pt2(i * grid_width, img_height);
        cv::line(out_img, pt1, pt2, cv::Scalar(255, 0, 0));
    }
    for (int i = 1; i < grid_col; ++i) {
        cv::Point pt1(i * grid_width + img_width, 0);
        cv::Point pt2(i * grid_width + img_width, img_height);
        cv::line(out_img, pt1, pt2, cv::Scalar(255, 0, 0));
    }

    // Colors for different features.
    cv::Scalar tracked(0, 255, 0);
    cv::Scalar new_feature(0, 0, 255);

    // Draw tracked features.
    for (const auto &id : prev_ids) {
        if (prev_cam0_points.find(id) != prev_cam0_points.end() &&
            curr_cam0_points.find(id) != curr_cam0_points.end()) {
            cv::Point2f prev_pt0 = cv::Point2f(prev_cam0_points[id].x, prev_cam0_points[id].y);
            cv::Point2f prev_pt1 = cv::Point2f(prev_cam1_points[id].x+img_width, prev_cam1_points[id].y);
            cv::Point2f curr_pt0 = cv::Point2f(curr_cam0_points[id].x, curr_cam0_points[id].y);
            cv::Point2f curr_pt1 = cv::Point2f(curr_cam1_points[id].x+img_width, curr_cam1_points[id].y);

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
        mynt::Point2f pt00 = new_cam0_point.second;
        mynt::Point2f pt01 = curr_cam1_points[new_cam0_point.first];

        cv::Point2f pt0 = cv::Point2f(pt00.x, pt00.y);
        cv::Point2f pt1 = cv::Point2f(pt01.x+img_width, pt01.y);

        cv::circle(out_img, pt0, 3, new_feature, -1);
        cv::circle(out_img, pt1, 3, new_feature, -1);
    }

    cv::resize(out_img, out_img, cv::Size(out_img.cols * 0.5, out_img.rows * 0.5));
    cv::namedWindow("Feature");
    cv::imshow("Feature", out_img);
    cv::waitKey(5);

    return;
}

int main() {
    std::cout << "start run_euroc..." << std::endl;

    std::string file_cam_imu = "../config/camchain-imucam-euroc.yaml";
    std::string euroc_dir = "/home/cg/projects/datasets/V1_01_easy/mav0/";
    int num_cams = 2;
    double timestamp_first = 0.0;

    system_ptr.reset(new mynt::System(file_cam_imu));

#if ONLY_GL
    glwindow::SceneWindow scene(640, 480, "Trajectory Viewer GL");
#else
    // init viewer, create pangolin window and plot the trajectory
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
#endif

    // image data
    std::vector<std::vector<std::pair<double, std::string>>> data_img(num_cams);
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
            std::string imgname = s.substr(0, s.size() - 1);
            data_img[n].push_back(std::make_pair(stamp_ns, imgname));
        }
        cam_file.close();
    }
    timestamp_first = data_img[0][0].first * 1e-9; // to seconds;

    if (num_cams == 2)
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
    while (idx_img < imgs_size) {
        mynt::Image imgs[num_cams];
        for (int j = 0; j < num_cams; ++j) {
            imgs[j].time_stamp = data_img[j][idx_img].first * 1e-9; // to seconds;
            std::string img_path = euroc_dir + "/cam" + std::to_string(j) + "/data/" + data_img[j][idx_img].second;
            cv::Mat mat_img = cv::imread(img_path, 0);
            mynt::YImg8 yimg(mat_img.rows, mat_img.cols);
            memcpy(yimg.data(), mat_img.data, yimg.size().area());
            imgs[j].image = yimg;
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
            mynt::Vector3 gyr;
            for (int j = 0; j < 3; ++j) {
                std::getline(stream, s, ',');
                gyr[j] = std::stof(s);
            }
            mynt::Vector3 acc;
            for (int j = 0; j < 3; ++j) {
                std::getline(stream, s, ',');
                acc[j] = std::stof(s);
            }
            // std::cout << "imu: " << gyr.transpose() << ", " << acc.transpose() << std::endl;

            boost::shared_ptr<mynt::Imu> imu(new mynt::Imu);
            imu->time_stamp = stamp_ns * 1e-9; // to seconds
            imu->angular_velocity = gyr;
            imu->linear_acceleration = acc;

            system_ptr->imu_callback(imu);

            t_imu = imu->time_stamp;

        } while (t_imu <= t_img);

        bool is_draw = true;
        system_ptr->stereo_callback(imgs[0], imgs[1], is_draw);

        if(is_draw) {
            draw_features_stereo(imgs[0].image, imgs[1].image,
                               system_ptr->imgproc_ptr_->prev_ids_,
                               system_ptr->imgproc_ptr_->prev_cam0_points_,
                               system_ptr->imgproc_ptr_->prev_cam1_points_,
                               system_ptr->imgproc_ptr_->curr_cam0_points_,
                               system_ptr->imgproc_ptr_->curr_cam1_points_,
                               system_ptr->imgproc_ptr_->processor_config.grid_row,
                               system_ptr->imgproc_ptr_->processor_config.grid_col);
        }

        system_ptr->backend_callback();

#if ONLY_GL
        if (scene.win.alive()) {
            if (scene.start_draw()) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
                glColor3f(0, 0, 1);

                glColor3f(255, 0, 0);
                glLineWidth(2);
                glBegin(GL_LINES);
                int size_path = system_ptr->path_to_draw_.size();
                for (int i = 0; i < size_path - 1; ++i) {
                    glVertex3f(system_ptr->path_to_draw_[i][0], system_ptr->path_to_draw_[i][1],
                               system_ptr->path_to_draw_[i][2]);
                    glVertex3f(system_ptr->path_to_draw_[i + 1][0], system_ptr->path_to_draw_[i + 1][1],
                               system_ptr->path_to_draw_[i + 1][2]);
                }
                glEnd();

                glColor3f(0, 0, 255);
                glPointSize(1.f);
                glBegin(GL_POINTS);
                int size_points = system_ptr->points3d_to_draw_.size();
                for (int i = 0; i < size_points; ++i) {
                    glVertex3f(system_ptr->points3d_to_draw_[i].x, system_ptr->points3d_to_draw_[i].y,
                               system_ptr->points3d_to_draw_[i].z);
                }
                glEnd();

                scene.finish_draw();
            }
        }
#else
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
        int size_path = system_ptr->path_to_draw_.size();
        for (int i = 0; i < size_path - 1; ++i) {
            glVertex3f(system_ptr->path_to_draw_[i][0], system_ptr->path_to_draw_[i][1],
                       system_ptr->path_to_draw_[i][2]);
            glVertex3f(system_ptr->path_to_draw_[i + 1][0], system_ptr->path_to_draw_[i + 1][1],
                       system_ptr->path_to_draw_[i + 1][2]);
        }
        glEnd();

        glColor3f(0, 0, 255);
        glPointSize(1.f);
        glBegin(GL_POINTS);
        int size_points = system_ptr->points3d_to_draw_.size();
        for (int i = 0; i < size_points; ++i) {
            glVertex3f(system_ptr->points3d_to_draw_[i].x, system_ptr->points3d_to_draw_[i].y,
                       system_ptr->points3d_to_draw_[i].z);
        }
        glEnd();

        pangolin::FinishFrame();
#endif
        idx_img++;
    }

    imu_file.close();

#if ONLY_GL
#else
    pangolin::QuitAll();
#endif

    std::cout << "end run_euroc..." << std::endl;

    return 0;
}

