#pragma once 
#include <tf2/LinearMath/Vector3.h>
#include <string> 
#include <nav_msgs/msg/occupancy_grid.hpp>

enum CellState : int8_t {Occupied = 100, Free = 0};
class Map
{
public:
    bool readFile(const std::string& mapYAMLfilepath);

    CellState at(int i, int j) const;
    CellState at(const tf2::Vector3& point) const;
    nav_msgs::msg::OccupancyGrid asOccupancyGrid() const;

private:
    tf2::Vector3 origin;
    double resolution;
    int width, height;
    std::vector<CellState> occupancyGrid;
};