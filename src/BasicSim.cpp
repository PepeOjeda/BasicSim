#include <basic_sim/core/Logging.hpp>
#include <basic_sim/BasicSim.hpp>
#include <basic_sim/ConvertYAML.hpp>
#include <filesystem>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicSim>();

    rclcpp::Rate rate(node->getSpeed() / node->getDeltaTime());
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->Update();
        rate.sleep();
    }

    return 0;
}

BasicSim::BasicSim() : Node("basic_sim")
{
    clockPub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).best_effort());
    mapPub = create_publisher<nav_msgs::msg::OccupancyGrid>("/basic_sim/map", rclcpp::QoS(1).transient_local());
    deltaTime = declare_parameter<double>("deltaTime", 0.1);
    speed = declare_parameter<double>("speed", 1);

    auto file = declare_parameter<std::string>("worldFile", "");
    parseFile(file);
    startTime = now();
}

void BasicSim::Update()
{
    for (Robot& robot : robots)
        robot.OnUpdate(deltaTime);
    publishClock();
}

void BasicSim::publishClock()
{
    rosgraph_msgs::msg::Clock msg;
    rclcpp::Duration ellapsed = (now() - startTime);
    msg.clock = rclcpp::Time{(int64_t)(speed * ellapsed.nanoseconds())};
    lastSentClockMsg = msg.clock;
    clockPub->publish(msg);
}

void BasicSim::parseFile(std::string& filepath)
{
    auto fail = [] {
        rclcpp::shutdown();
        exit(-1);
    };

    if (!std::filesystem::exists(filepath))
    {
        BS_ERROR("File %s does not exist! Closing BasicSim.", filepath.c_str());
        fail();
    }
    std::filesystem::path bs_yamlPath(filepath);


    YAML::Node bs_yaml = YAML::LoadFile(filepath);
    
    // map image
    YAML::Node mapYAML = bs_yaml["map"];
    if (!mapYAML)
    {
        BS_ERROR("Entry 'map' not present in %s", filepath.c_str());
        fail();
    }

        //if the map's YAML file is specified with a relative path, interpret it as being relative to the BasicSim yaml
    std::filesystem::path mapYamlPath(mapYAML.as<std::string>());
    if(mapYamlPath.is_relative())
        mapYamlPath = bs_yamlPath.parent_path()/mapYamlPath;
    if (!map.readFile(mapYamlPath))
    {
        BS_ERROR("Failure parsing map image!");
        fail();
    }
    mapPub->publish(map.asOccupancyGrid());

    //robots
    YAML::Node robotsList = bs_yaml["robots"];
    if (!robotsList)
    {
        BS_ERROR("Entry 'robots' not present in %s", filepath.c_str());
        fail();
    }

    robots.reserve(robotsList.size());
    for (YAML::Node robot : robotsList)
    {
        tf2::Transform initialPose;

        initialPose.setOrigin(robot["position"].as<tf2::Vector3>());
        initialPose.setRotation(tf2::Quaternion({0, 0, 1}, robot["angle"].as<double>()));

        robots.emplace_back(std::make_shared<rclcpp::Node>(robot["name"].as<std::string>()), initialPose, this);
    }
}
