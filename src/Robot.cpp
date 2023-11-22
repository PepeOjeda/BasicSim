#include <basic_sim/Robot.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <basic_sim/BasicSim.hpp>
#include <basic_sim/Logging.hpp>

Robot::Robot(std::string& name, const tf2::Transform& startingPose, const BasicSim* sim, const std::vector<LaserSensorDescription>& lasers)
    : m_name(name), m_currentTransform(startingPose), m_sim(sim)
{
    m_node = std::make_shared<rclcpp::Node>(name);

    std::string cmd_vel_topic = "/" + m_name + "/cmd_vel";
    m_cmd_velSub = m_node->create_subscription<geo::Twist>(cmd_vel_topic, 1, std::bind(&Robot::cmd_velCallback, this, std::placeholders::_1));

    m_robotBaseBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);

    m_odomGroundTruthBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node);
    geo::TransformStamped mapToOdom;
    mapToOdom.header.frame_id = "map";
    mapToOdom.child_frame_id = "odom";
    m_odomGroundTruthBroadcaster->sendTransform(mapToOdom);

    m_laserScanners.reserve(lasers.size());
    for(const auto& laserDesc : lasers)
    {
        LaserSensor& sensor = m_laserScanners.emplace_back(laserDesc, &m_sim->map);
        sensor.publisher = m_node->create_publisher<sensor_msgs::msg::LaserScan>(m_name+"/"+laserDesc.name, rclcpp::QoS(1));
        sensor.frame_id = getRobotFrameId(); //TODO maybe add the option to attach the scanner to a different TF frame.
    }

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
    UpdatePose(deltaTime);
    UpdateSensors(deltaTime);
}

void Robot::UpdatePose(float deltaTime)
{
    tf2::Transform nextTransform;

    // we are applying rotation first, displacement later
    // not calculating the arc from applying both simultaneously, because deltaTime is very small
    tf2::Quaternion rotation({0, 0, 1}, m_currentTwist.angular.z * deltaTime);
    tf2::Vector3 linearMovement;
    tf2::fromMsg(m_currentTwist.linear, linearMovement);
    linearMovement *= deltaTime;
    tf2::Transform movement(rotation, tf2::quatRotate(rotation, linearMovement));

    nextTransform.mult(m_currentTransform, movement);

    CellState cellState = m_sim->map.at(nextTransform.getOrigin());
    if (cellState == CellState::Free)
        m_currentTransform = nextTransform;

    // send TF
    geo::TransformStamped odomToBase;
    odomToBase.header.frame_id = "odom";
    odomToBase.header.stamp = m_sim->getCurrentTime();
    odomToBase.child_frame_id = getRobotFrameId();
    odomToBase.transform = tf2::toMsg(m_currentTransform);
    m_robotBaseBroadcaster->sendTransform(odomToBase);
}

void Robot::UpdateSensors(float deltaTime)
{
    for(LaserSensor& sensor : m_laserScanners)
    {
        tf2::Vector3 forward = tf2::quatRotate(m_currentTransform.getRotation(), {1,0,0});
        auto msg = sensor.scanner.Scan(m_currentTransform.getOrigin(), forward);

        msg.header.frame_id = sensor.frame_id; 
        msg.header.stamp = m_sim->getCurrentTime(); 
        msg.scan_time = deltaTime;
        sensor.publisher->publish(msg);
    }
}

std::string Robot::getRobotFrameId()
{
    return m_name + "_base_link";
}
