#pragma once
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/publisher.hpp>

#include <basic_sim/Map.hpp>

struct LaserSensorDescription
{
    std::string name; 
    float minAngleRad, maxAngleRad;
    float minDistance, maxDistance;
    float angleResolutionRad;
    float noiseStdDev = 0;
};

class LaserScanner
{
public:
    LaserScanner(const LaserSensorDescription& desc, const Map* map);
    sensor_msgs::msg::LaserScan Scan(const tf2::Vector3& position, const tf2::Vector3& forwardDirection);

private:
    LaserSensorDescription description;
    DDA::_2D::Map<CellState> DDA_map;
};

struct LaserSensor
{
    LaserSensor() = delete;
    LaserSensor(const LaserSensor& other) = delete;
    LaserSensor(LaserSensor&& other) = default;
    LaserSensor(const LaserSensorDescription& description, const Map* map)
        : scanner(description, map)
    {}

    std::string frame_id;
    LaserScanner scanner;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
};