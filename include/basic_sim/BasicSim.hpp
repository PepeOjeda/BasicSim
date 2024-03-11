#pragma once
#include <basic_sim/Map.hpp>
#include <basic_sim/Robot.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

class BasicSim : public rclcpp::Node
{
public:
    BasicSim();
    void Update();

    double getDeltaTime() const {return deltaTime;}
    double getSpeed() const {return speed;}
    rclcpp::Time getCurrentTime() const {return currentTime;}
    
    Map map;
    
protected:
    double deltaTime;
    double speed;
    rclcpp::Time startTime;
    rclcpp::Time currentTime;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr resetRobotsSub;
    
    // It is important that the vector is only resized once, before creating any robots
    // otherwise, the robots will be moved by the resize operation, and the bound callbacks will continue to use the original address
    // in fact, modifying the list of robots at all after the initial file parsing is not allowed, as it will cause similar address problems
    std::vector<Robot> robots;

    void parseFile(std::string& filepath);
    void parseRobot(YAML::Node robotYAML);
    void publishClock();
	void ResetRobotsToStartingPose(std_msgs::msg::String::SharedPtr msg);
};