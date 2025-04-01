#pragma once
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Utils
{
    inline geometry_msgs::msg::Pose transformToPose(const geometry_msgs::msg::Transform& tf)
    {
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf.rotation;
        pose.position.x = tf.translation.x;
        pose.position.y = tf.translation.y;
        pose.position.z = tf.translation.z;
        return pose;
    }
}