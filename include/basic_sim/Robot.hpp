#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include <basic_sim/LaserScanner.hpp>

class BasicSim;

namespace geo = geometry_msgs::msg;

class Robot
{
public:
    Robot() = delete;
    Robot(const Robot&) = delete;
    Robot(Robot&&) = default;
    Robot(std::string& name, const tf2::Transform& startingPose, const BasicSim* sim, const std::vector<LaserSensorDescription>& lasers);

    void OnUpdate(float deltaTime);

private:
    const BasicSim* m_sim;
    rclcpp::Node::SharedPtr m_node;
    std::string m_name;
    tf2::Transform m_currentTransform;
    geo::Twist m_currentTwist;

    std::vector<LaserSensor> m_laserScanners;

    rclcpp::Subscription<geo::Twist>::SharedPtr m_cmd_velSub;
    rclcpp::Subscription<geo::PoseWithCovarianceStamped>::SharedPtr m_resetPoseSub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_odomGroundTruthBroadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_robotBaseBroadcaster;
    rclcpp::Publisher<geo::PoseWithCovarianceStamped>::SharedPtr m_posePub;

    void cmd_velCallback(geo::Twist::SharedPtr msg);
    void UpdatePose(float deltaTime);
    void UpdateSensors(float deltaTime);
    std::string getRobotFrameId();
    void resetPoseCallback(geo::PoseWithCovarianceStamped::SharedPtr msg);
};