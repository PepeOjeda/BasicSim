#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

class BasicSim;

namespace geo = geometry_msgs::msg;

class Robot
{
    public:

    Robot() = delete;
    Robot(const Robot&) = delete;
    Robot(Robot&&) = default;
    Robot(rclcpp::Node::SharedPtr node, const tf2::Transform& startingPose, const BasicSim* sim);

    void OnUpdate(float deltaTime);

    private:
    const BasicSim* m_sim;
    rclcpp::Node::SharedPtr m_node;
    std::string m_name;
    tf2::Transform m_currentTransform;
    geo::Twist m_currentTwist;

    rclcpp::Subscription<geo::Twist>::SharedPtr m_cmd_velSub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_odomGroundTruthBroadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_robotBaseBroadcaster;

    void cmd_velCallback(geo::Twist::SharedPtr msg);
};