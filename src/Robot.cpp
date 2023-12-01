#include <basic_sim/Robot.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <basic_sim/BasicSim.hpp>
#include <basic_sim/Logging.hpp>

Robot::Robot(std::string& name, const tf2::Transform& startingPose, const BasicSim* sim, const std::vector<LaserSensorDescription>& lasers)
    : m_name(name), m_currentTransform(startingPose), m_startingTransform(startingPose), m_sim(sim)
{
    m_node = std::make_shared<rclcpp::Node>(name);

    std::string cmd_vel_topic = "/" + m_name + "/cmd_vel";
    m_cmd_velSub = m_node->create_subscription<geo::Twist>(cmd_vel_topic, 1, std::bind(&Robot::cmd_velCallback, this, std::placeholders::_1));


    m_robotBaseBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(m_node);
    m_posePub = m_node->create_publisher<geo::PoseWithCovarianceStamped>("/"+m_name+"/ground_truth", rclcpp::QoS(5));
    m_resetPoseSub = m_node->create_subscription<geo::PoseWithCovarianceStamped>("/"+m_name+"/initialpose", rclcpp::QoS(1), std::bind(&Robot::resetPoseCallback, this, std::placeholders::_1));
    m_odomPub = m_node -> create_publisher<nav_msgs::msg::Odometry>("/"+m_name+"/odom", rclcpp::QoS(1));
    
    // publish static map_odom TF (published only once)

    //TODO Right now the odom frame is perfectly aligned with the map frame. This is fine, since we don't actually need to do anything with it
    // however, it might be nice to do the standard ROS thing and place the origin of odom at the robot's initial pose
    // if we make this change, we need to modify the message that is published to the odom topic in UpdatePose() accordingly 
    m_odomGroundTruthBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(m_node);
    geo::TransformStamped mapToOdom;
    mapToOdom.header.frame_id = "map";
    mapToOdom.child_frame_id = m_name + "_odom";
    m_odomGroundTruthBroadcaster->sendTransform(mapToOdom);


    // create the sensors specified in the YAML
    m_laserScanners.reserve(lasers.size());
    for(const auto& laserDesc : lasers)
    {
        LaserSensor& sensor = m_laserScanners.emplace_back(laserDesc, &m_sim->map);
        sensor.publisher = m_node->create_publisher<sensor_msgs::msg::LaserScan>(m_name+"/"+laserDesc.name, rclcpp::QoS(1));
        sensor.frame_id = getRobotFrameId(); //TODO maybe add the option to attach the scanner to a different TF frame.
    }

    m_currentVelocityMsg.simTimeStamp = m_sim->getCurrentTime();
    BS_INFO("Created robot %s", m_name.c_str());
}


void Robot::OnUpdate(float deltaTime)
{
    rclcpp::spin_some(m_node);
    UpdatePose(deltaTime);
    UpdateSensors(deltaTime);
}

void Robot::UpdatePose(float deltaTime)
{
    if(m_sim->getCurrentTime().seconds()-m_currentVelocityMsg.simTimeStamp.seconds() > 0.5)
        m_currentVelocityMsg.Reset();

    tf2::Transform nextTransform;
    // we are applying rotation first, displacement later
    // not calculating the arc from applying both simultaneously, because deltaTime is very small
    tf2::Quaternion rotation({0, 0, 1}, m_currentVelocityMsg.twist.angular.z * deltaTime);
    tf2::Vector3 linearMovement;
    tf2::fromMsg(m_currentVelocityMsg.twist.linear, linearMovement);
    linearMovement *= deltaTime;
    tf2::Transform movement(rotation, tf2::quatRotate(rotation, linearMovement));

    nextTransform.mult(m_currentTransform, movement);

    CellState cellState = m_sim->map.at(nextTransform.getOrigin());
    if (cellState == CellState::Free)
        m_currentTransform = nextTransform;

    // send TF
    geo::TransformStamped odomToBase;
    odomToBase.header.frame_id = m_name + "_odom";
    odomToBase.header.stamp = m_sim->getCurrentTime();
    odomToBase.child_frame_id = getRobotFrameId();
    odomToBase.transform = tf2::toMsg(m_currentTransform);
    m_robotBaseBroadcaster->sendTransform(odomToBase);

    //publish PoseWithCovarianceStamped msg to /(robot)/ground_truth
    geo::PoseWithCovarianceStamped poseMsg;
    poseMsg.header.frame_id = "map";
    poseMsg.header.stamp = m_sim->getCurrentTime();
    poseMsg.pose.pose.position.x = odomToBase.transform.translation.x;
    poseMsg.pose.pose.position.y = odomToBase.transform.translation.y;
    poseMsg.pose.pose.position.z = odomToBase.transform.translation.z;
    poseMsg.pose.pose.orientation = odomToBase.transform.rotation;
    m_posePub->publish(poseMsg);

    //publish Odometry msg
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header.frame_id = m_name + "_odom";
    odomMsg.header.stamp = m_sim->getCurrentTime();
    odomMsg.child_frame_id = getRobotFrameId();
    odomMsg.pose = poseMsg.pose; //TODO this only works because map and odom are the same frame! If we change the odom frame we will need to transform the (map frame) pose to it before sending
    odomMsg.twist.twist = m_currentVelocityMsg.twist;
    m_odomPub->publish(odomMsg);
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

void Robot::resetPoseCallback(geo::PoseWithCovarianceStamped::SharedPtr msg)
{
#define IGNORE_MSG_Z 1 //because rviz's initialpose tool always sets z to 0, and that's quite annoying. At some point there should probably be a custom rviz tool for this
#if IGNORE_MSG_Z
    tf2::Vector3 position = {msg->pose.pose.position.x, msg->pose.pose.position.y, m_currentTransform.getOrigin().z()};
#else
    tf2::Vector3 position = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
#endif
    CellState cellState = m_sim->map.at(position);
    if (cellState == CellState::Free)
        m_currentTransform.setOrigin(position);
    else
        BS_ERROR("Trying to set robot %s to position (%.2f, %.2f, %.2f), but it is not free!", m_name.c_str(), position.x(), position.y(), position.z());
}

void Robot::ResetToStartingPose()
{
	BS_INFO("Reset robot %s to starting position.", m_name.c_str());
	m_currentTransform = m_startingTransform;
	m_currentVelocityMsg.Reset();
}

void Robot::cmd_velCallback(geo::Twist::SharedPtr msg)
{
    m_currentVelocityMsg.twist = *msg;
    m_currentVelocityMsg.simTimeStamp = m_sim->getCurrentTime();
}

void Robot::VelocityMsg::Reset()
{
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
}

