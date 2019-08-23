//
// Created by cg on 8/20/19.
//

#ifndef MSCKF_VIO_MYNT_CONFIG_IO_H
#define MSCKF_VIO_MYNT_CONFIG_IO_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace YAML {
    // Vector2d
    template<>
    struct convert<Eigen::Vector2d> {
        static bool decode(const Node& node, Eigen::Vector2d& rhs) {
            if(!node.IsSequence() || node.size() != 2) {
                return false;
            }
            rhs[0] = node[0].as<double>();
            rhs[1] = node[1].as<double>();
            return true;
        }
    };

    // Vector3d
    template<>
    struct convert<Eigen::Vector3d> {
        static bool decode(const Node& node, Eigen::Vector3d& rhs) {
            if(!node.IsSequence() || node.size() != 3) {
                return false;
            }
            rhs[0] = node[0].as<double>();
            rhs[1] = node[1].as<double>();
            rhs[2] = node[2].as<double>();
            return true;
        }
    };

    // Vector4d
    template<>
    struct convert<Eigen::Vector4d> {
        static Node encode(const Eigen::Vector4d& rhs) {
            Node node;
            node.push_back(rhs[0]);
            node.push_back(rhs[1]);
            node.push_back(rhs[2]);
            node.push_back(rhs[3]);
            return node;
        }

        static bool decode(const Node& node, Eigen::Vector4d& rhs) {
            if(!node.IsSequence() || node.size() != 4) {
                return false;
            }
            rhs[0] = node[0].as<double>();
            rhs[1] = node[1].as<double>();
            rhs[2] = node[2].as<double>();
            rhs[3] = node[3].as<double>();
            return true;
        }
    };

    // Matrix4d
    template<>
    struct convert<Eigen::Matrix4d> {
        static bool decode(const Node& node, Eigen::Matrix4d& rhs) {
            if(!node.IsSequence() || node.size() != 16) {
                return false;
            }
//            for(int i=0; i<16; ++i)
//                rhs.data()[i] = node[i].as<double>();

            for(int i=0; i<4; ++i)
                for(int j=0; j<4; ++j)
                    rhs(i,j) = node[j+i*4].as<double>();

            return true;
        }
    };
}

class ConfigIO {
public:
    ConfigIO() {}
};

#endif //MSCKF_VIO_MYNT_CONFIG_IO_H
