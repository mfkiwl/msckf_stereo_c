//
// Created by cg on 8/21/19.
//

#ifndef MSCKF_VIO_MYNT_SYSTEM_H
#define MSCKF_VIO_MYNT_SYSTEM_H

#include <string>
#include <mutex>

#include "image_processor.h"
#include "msckf_vio.h"

namespace mynt {
    class System {
    public:
        System(std::string file_cam_imu);
        ~System();

        void stereo_callback(const mynt::Image &cam0_img, const mynt::Image &cam1_img);

        void imu_callback(const mynt::ImuConstPtr &msg);

        void backend_callback();

        void draw();

        typedef boost::shared_ptr<System> Ptr;
        typedef boost::shared_ptr<const System> ConstPtr;

    private:
        YAML::Node cfg_cam_imu_;

        boost::shared_ptr<CameraMeasurement> feature_msg_ptr_;

        mynt::ImageProcessorPtr imgproc_ptr_;
        mynt::MsckfVioPtr msckfvio_ptr_;

        bool is_start_backend_;

        std::mutex mt_feature_;
    };

    typedef System::Ptr SystemPtr;
    typedef System::ConstPtr SystemConstPtr;
}

#endif //MSCKF_VIO_MYNT_SYSTEM_H
