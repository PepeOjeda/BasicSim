#pragma once

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

namespace YAML
{
    template <> struct convert<tf2::Vector3>
    {
        static Node encode(const tf2::Vector3& rhs)
        {
            Node node;
            node.push_back(rhs.x());
            node.push_back(rhs.y());
            node.push_back(rhs.z());
            return node;
        }

        static bool decode(const Node& node, tf2::Vector3& rhs)
        {
            if (!node.IsSequence() || node.size() != 3)
            {
                return false;
            }

            rhs.setX(node[0].as<double>());
            rhs.setY(node[1].as<double>());
            rhs.setZ(node[2].as<double>());
            return true;
        }
    };

    template <> struct convert<tf2::Quaternion>
    {
        static Node encode(const tf2::Quaternion& rhs)
        {
            Node node;
            node.push_back(rhs.x());
            node.push_back(rhs.y());
            node.push_back(rhs.z());
            node.push_back(rhs.w());
            return node;
        }

        static bool decode(const Node& node, tf2::Quaternion& rhs)
        {
            if (!node.IsSequence() || node.size() != 4)
            {
                return false;
            }

            rhs.setX(node[0].as<double>());
            rhs.setY(node[1].as<double>());
            rhs.setZ(node[2].as<double>());
            rhs.setW(node[3].as<double>());
            return true;
        }
    };
} // namespace YAML