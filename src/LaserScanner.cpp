#include <basic_sim/LaserScanner.hpp>

LaserScanner::LaserScanner(float _minAngleRad, float _maxAngleRad, float _minDistance, float _maxDistance, float _angleResolutionRad, const Map* map)
    : minAngleRad(_minAngleRad), maxAngleRad(_maxAngleRad), minDistance(_minDistance), maxDistance(_maxDistance),
      angleResolutionRad(_angleResolutionRad)
{
    while (minAngleRad > maxAngleRad)
        maxAngleRad += 2 * M_PI;

    DDA_map = map->asDDAMap();
}

LaserScanner::LaserScanner(const LaserSensorDescription& desc, const Map* map)
    : minAngleRad(desc.minAngleRad), maxAngleRad(desc.maxAngleRad), minDistance(desc.minDistance), maxDistance(desc.maxDistance),
      angleResolutionRad(desc.angleResolutionRad)
{
    while (minAngleRad > maxAngleRad)
        maxAngleRad += 2 * M_PI;

    DDA_map = map->asDDAMap();
}

sensor_msgs::msg::LaserScan LaserScanner::Scan(const tf2::Vector3& position, const tf2::Vector3& forwardDirectionTF)
{
    int numberOfMeasurements = (maxAngleRad - minAngleRad) / angleResolutionRad;

    sensor_msgs::msg::LaserScan msg;
    msg.angle_min = minAngleRad;
    msg.angle_max = maxAngleRad;
    msg.angle_increment = angleResolutionRad;
    msg.time_increment = 0;
    msg.range_min = minDistance;
    msg.range_max = maxDistance;
    msg.ranges.resize(numberOfMeasurements, 0);

    DDA::Vector2 start(position.x(), position.y());
    DDA::Vector2 forward(forwardDirectionTF.x(), forwardDirectionTF.y());

    #pragma omp parallel for
    for (int i = 0; i < numberOfMeasurements; i++)
    {
        float angle = minAngleRad + i * angleResolutionRad;
        DDA::_2D::RayCastInfo info =
            DDA::_2D::castRay<CellState>(start, forward.rotate(angle), maxDistance, DDA_map, [](CellState c) { return c == CellState::Free; });

        if (info.hitSomething)
            msg.ranges[i] = info.distance;
    }

    return msg;
}
