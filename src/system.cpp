//
// Created by cg on 8/21/19.
//

#include "system.h"

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
            mt_feature_.unlock();
            usleep(30000);
        }
    }
}

