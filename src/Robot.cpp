#include <basic_sim/Robot.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <basic_sim/BasicSim.hpp>
#include <basic_sim/core/Logging.hpp>

Robot::Robot(rclcpp::Node::SharedPtr node, const tf2::Transform& startingPose, const BasicSim* sim)
    : m_node(node), m_name(node->get_name()), m_currentTransform(startingPose), m_sim(sim)
{
    std::string cmd_vel_topic = "/" + m_name + "/cmd_vel";
    m_cmd_velSub = m_node->create_subscription<geo::Twist>(cmd_vel_topic, 1, std::bind(&Robot::cmd_velCallback, this, std::placeholders::_1));
 
    m_robotBaseBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);

    m_odomGroundTruthBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node);
    geo::TransformStamped mapToOdom;
    mapToOdom.header.frame_id = "map";
    mapToOdom.child_frame_id = "odom";
    m_odomGroundTruthBroadcaster->sendTransform(mapToOdom);

    BS_INFO("Created robot %s", m_name.c_str());

}

void Robot::cmd_velCallback(geo::Twist::SharedPtr msg)
{
    m_currentTwist.linear = msg->linear;
    m_currentTwist.angular = msg->angular;
}

void Robot::OnUpdate(float deltaTime)
{
    rclcpp::spin_some(m_node);
    tf2::Transform nextTransform;

    //we are applying rotation first, displacement later
    //not calculating the arc from applying both simultaneously, because deltaTime is very small
    tf2::Quaternion rotation({0,0,1}, m_currentTwist.angular.z * deltaTime);
    tf2::Vector3 linearMovement;
    tf2::fromMsg(m_currentTwist.linear, linearMovement);
    linearMovement*= deltaTime;
    tf2::Transform movement(rotation, tf2::quatRotate(rotation, linearMovement));
    
    nextTransform.mult(m_currentTransform, movement);

    CellState cellState = m_sim->map.at(nextTransform.getOrigin());
    if(cellState == CellState::Free)
        m_currentTransform = nextTransform;

    //send TF
    geo::TransformStamped odomToBase;
    odomToBase.header.frame_id = "odom";
    odomToBase.header.stamp = m_sim->getCurrentTime();
    odomToBase.child_frame_id = m_name + "_base_link";
    odomToBase.transform = tf2::toMsg(m_currentTransform);
    m_robotBaseBroadcaster->sendTransform(odomToBase);
}

