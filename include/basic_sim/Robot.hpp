#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <basic_sim/LaserScanner.hpp>

class BasicSim;

namespace geo = geometry_msgs::msg;

struct RobotDescription
{
    const std::string& name;
    const tf2::Transform& startingPose;
    float radius;
    const BasicSim* sim;
    const std::vector<LaserSensorDescription>& lasers;
    bool publishOdom;
};

class Robot
{
public:
    Robot() = delete;
    Robot(const Robot&) = delete;
    Robot(Robot&&) = default;
    Robot(const RobotDescription& description);

    void OnUpdate(float deltaTime);
    void ResetToStartingPose();

    const std::string m_name;

private:
    const BasicSim* m_sim;
    rclcpp::Node::SharedPtr m_node;
    const tf2::Transform m_startingTransform;
    tf2::Transform m_currentTransformMapFrame;
    tf2::Transform m_mapToOdom; // for turning map frame pose into odom frame pose
    float m_radius;
    bool publishOdom;

    std::vector<LaserSensor> m_laserScanners;

    rclcpp::Subscription<geo::Twist>::SharedPtr m_cmd_velSub;
    rclcpp::Subscription<geo::PoseWithCovarianceStamped>::SharedPtr m_resetPoseSub;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_odomGroundTruthBroadcaster;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_robotBaseBroadcaster;
    rclcpp::Publisher<geo::PoseWithCovarianceStamped>::SharedPtr m_posePub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odomPub;

    void UpdatePose(float deltaTime);
    void UpdateSensors(float deltaTime);
    void PublishPoseAndOdom(tf2::Transform movement, float deltaTime);
    std::string getRobotFrameId();

    bool canBeAt(const tf2::Vector3& position) const;

    struct VelocityMsg
    {
        rclcpp::Time simTimeStamp;
        geo::Twist twist;
        void Reset();
    } m_currentVelocityMsg;
    void cmd_velCallback(geo::Twist::SharedPtr msg);
    void resetPoseCallback(geo::PoseWithCovarianceStamped::SharedPtr msg);
};