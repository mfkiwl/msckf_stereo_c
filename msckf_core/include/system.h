//
// Created by cg on 8/21/19.
//

#ifndef MSCKF_VIO_MYNT_SYSTEM_H
#define MSCKF_VIO_MYNT_SYSTEM_H

#include <string>
#include <mutex>

#include "image_processor.h"
#include "msckf_vio.h"
#include "maths/vector.h"

namespace mynt {
    class System {
    public:
        System(std::string file_cam_imu);
        ~System();

        void stereo_callback(const mynt::Image &cam0_img, const mynt::Image &cam1_img, bool is_draw = false);

        void imu_callback(const mynt::ImuConstPtr &msg);

        void backend_callback();

        std::vector<mynt::Vector3> path_to_draw_;

        std::vector<mynt::Point3f> points3d_to_draw_;

        mynt::ImageProcessorPtr imgproc_ptr_;

        typedef std::shared_ptr<System> Ptr;
        typedef std::shared_ptr<const System> ConstPtr;

    private:
        YAML::Node cfg_cam_imu_;

        std::shared_ptr<CameraMeasurement> feature_msg_ptr_;

        mynt::MsckfVioPtr msckfvio_ptr_;
    };

    typedef System::Ptr SystemPtr;
    typedef System::ConstPtr SystemConstPtr;
}

#endif //MSCKF_VIO_MYNT_SYSTEM_H
