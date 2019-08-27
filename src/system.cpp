//
// Created by cg on 8/21/19.
//

#include "system.h"

#include <pangolin/pangolin.h>
using namespace pangolin;

namespace mynt {

    System::System(std::string file_cam_imu) :
        feature_msg_ptr_(new CameraMeasurement),
        is_start_backend_(true) {
        cfg_cam_imu_ = YAML::LoadFile(file_cam_imu);

        // image_processor init
        imgproc_ptr_.reset(new mynt::ImageProcessor(cfg_cam_imu_));
        try {
            if (!imgproc_ptr_->initialize()) {
                throw std::runtime_error("RUNTIME ERROR: Cannot initialize Image Processor...");
            }
        } catch (...) {
            std::cerr << "Cannot initialize Image Processor..." << std::endl;
        }

        // MsckfVio
        msckfvio_ptr_.reset(new mynt::MsckfVio(cfg_cam_imu_));
        try {
            if (!msckfvio_ptr_->initialize()) {
                throw std::runtime_error("RUNTIME ERROR: Cannot initialize MsckfVio...");
            }
        } catch (...) {
            std::cerr << "Cannot initialize MsckfVio..." << std::endl;
        }
    }

    System::~System() {
        is_start_backend_ = false;

        pangolin::QuitAll();

        msckfvio_ptr_->resetCallback();
    }

    void System::stereo_callback(const mynt::Image &cam0_img, const mynt::Image &cam1_img) {
        mt_feature_.lock();
        imgproc_ptr_->stereoCallback(cam0_img, cam1_img);
        feature_msg_ptr_ = imgproc_ptr_->feature_msg_ptr_;
        mt_feature_.unlock();
    }

    void System::imu_callback(const mynt::ImuConstPtr &msg) {
        imgproc_ptr_->imuCallback(msg);
        msckfvio_ptr_->imuCallback(msg);
    }

    void System::backend_callback() {
        while(is_start_backend_) {
            mt_feature_.lock();
            msckfvio_ptr_->featureCallback(feature_msg_ptr_);
            path_to_draw_ = msckfvio_ptr_->get_path();
            mt_feature_.unlock();
            usleep(10000);
        }
    }

    void System::draw() {
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
            int nPath_size = path_to_draw_.size();
            for(int i = 0; i < nPath_size-1; ++i)
            {
                glVertex3f(path_to_draw_[i].x(), path_to_draw_[i].y(), path_to_draw_[i].z());
                glVertex3f(path_to_draw_[i+1].x(), path_to_draw_[i+1].y(), path_to_draw_[i+1].z());
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
}

