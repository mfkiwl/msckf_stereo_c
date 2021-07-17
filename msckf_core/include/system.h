//
// Created by cg on 8/21/19.
//

#ifndef MSCKF_SYSTEM_H
#define MSCKF_SYSTEM_H

#include <string>
#include <mutex>

#include "image_processor.h"
#include "msckf_vio.h"
#include "maths/vector.h"

namespace cg {
    class System {
    public:
        System(std::string file_cam_imu);
        ~System();

        void stereo_callback(const cg::Image &cam0_img, const cg::Image &cam1_img, bool is_draw = false);

        void imu_callback(const cg::ImuConstPtr &msg);

        void backend_callback();

        std::vector<cg::Vector3> path_to_draw_;

        std::vector<cg::Point3f> points3d_to_draw_;

        cg::ImageProcessorPtr imgproc_ptr_;

        typedef std::shared_ptr<System> Ptr;
        typedef std::shared_ptr<const System> ConstPtr;

    private:
        YAML::Node cfg_cam_imu_;

        std::shared_ptr<CameraMeasurement> feature_msg_ptr_;

        cg::MsckfVioPtr msckfvio_ptr_;
    };

    typedef System::Ptr SystemPtr;
    typedef System::ConstPtr SystemConstPtr;
}

#endif //MSCKF_SYSTEM_H
