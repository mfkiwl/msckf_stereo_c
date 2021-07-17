//
// Created by cg on 8/20/19.
//

#ifndef MSCKF_CONFIG_IO_H
#define MSCKF_CONFIG_IO_H

#include <yaml-cpp/yaml.h>

#include "maths/vector.h"
#include "maths/mat.h"

namespace YAML {
    // Vector2d
    template<>
    struct convert<cg::Vector2> {
        static bool decode(const Node& node, cg::Vector2& rhs) {
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
    struct convert<cg::Vector3> {
        static bool decode(const Node& node, cg::Vector3& rhs) {
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
    struct convert<cg::Vector4> {
        static Node encode(const cg::Vector4& rhs) {
            Node node;
            node.push_back(rhs[0]);
            node.push_back(rhs[1]);
            node.push_back(rhs[2]);
            node.push_back(rhs[3]);
            return node;
        }

        static bool decode(const Node& node, cg::Vector4& rhs) {
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
    struct convert<cg::Mat4> {
        static bool decode(const Node& node, cg::Mat4& rhs) {
            if(!node.IsSequence() || node.size() != 16) {
                return false;
            }

            for(int i=0; i<4; ++i)
                for(int j=0; j<4; ++j)
                    rhs(i,j) = node[j+i*4].as<double>();

            return true;
        }
    };
}

#endif //MSCKF_CONFIG_IO_H
