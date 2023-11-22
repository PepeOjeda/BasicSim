#include <basic_sim/LaserScanner.hpp>

LaserScanner::LaserScanner(float _minAngleRad, float _maxAngleRad, float _minDistance, float _maxDistance, float _angleResolutionRad, const Map* map)
    : minAngleRad(_minAngleRad), maxAngleRad(_maxAngleRad), minDistance(_minDistance), maxDistance(_maxDistance),
      angleResolutionRad(_angleResolutionRad)
{
    while(minAngleRad > maxAngleRad)
        maxAngleRad += 2*M_PI;
        
    DDA_map = map->asDDAMap();
}

LaserScanner::LaserScanner(const LaserSensorDescription& desc, const Map* map)
    : minAngleRad(desc.minAngleRad), maxAngleRad(desc.maxAngleRad), minDistance(desc.minDistance), maxDistance(desc.maxDistance),
      angleResolutionRad(desc.angleResolutionRad)
{
    while(minAngleRad > maxAngleRad)
        maxAngleRad += 2*M_PI;

    DDA_map = map->asDDAMap();
}

sensor_msgs::msg::LaserScan LaserScanner::Scan(const tf2::Vector3& position, const tf2::Vector3& forwardDirectionTF)
{
#define PARALLEL 1
#if PARALLEL
    struct Measurement
    {
        int index;
        float distance;
    };

    int numberOfMeasurements = (maxAngleRad - minAngleRad) / angleResolutionRad;

    std::vector<Measurement> measurements(numberOfMeasurements);

    sensor_msgs::msg::LaserScan msg;
    msg.angle_min = minAngleRad;
    msg.angle_max = maxAngleRad;
    msg.angle_increment = angleResolutionRad;
    msg.time_increment = 0;
    msg.range_min = minDistance;
    msg.range_max = maxDistance;
    msg.ranges.reserve(numberOfMeasurements);

    DDA::Vector2 start(position.x(), position.y());
    DDA::Vector2 forward(forwardDirectionTF.x(), forwardDirectionTF.y());

    std::mutex mtx;
    #pragma omp parallel for
    for (int i = 0; i < numberOfMeasurements; i++)
    {
        measurements[i].index = i;
        float angle = minAngleRad + i * angleResolutionRad;
        DDA::_2D::RayCastInfo info =
            DDA::_2D::castRay<CellState>(start, forward.rotate(angle), maxDistance, DDA_map, [](CellState c) { return c == CellState::Free; });
        
        mtx.lock();
        if (info.hitSomething)
        {
            measurements[i].distance = info.distance;
        }
        else
        {
            measurements[i].distance = 0;
        }
        mtx.unlock();
    }

    std::sort(measurements.begin(), measurements.end(), [](const Measurement& a, const Measurement& b) { return a.index < b.index; });

    for(const Measurement& m : measurements)
        msg.ranges.push_back(m.distance);
    return msg;
#else
    int numberOfMeasurements = (maxAngleRad - minAngleRad) / angleResolutionRad;

    sensor_msgs::msg::LaserScan msg;
    msg.angle_min = minAngleRad;
    msg.angle_max = maxAngleRad;
    msg.angle_increment = angleResolutionRad;
    msg.time_increment = 0;
    msg.range_min = minDistance;
    msg.range_max = maxDistance;
    msg.ranges.reserve(numberOfMeasurements);

    DDA::Vector2 start(position.x(), position.y());
    DDA::Vector2 forward(forwardDirectionTF.x(), forwardDirectionTF.y());

    //for (float angle = minAngleRad; angle < maxAngleRad; angle += angleResolutionRad)
    
    //std::mutex mtx;
    //#pragma omp parallel for
    for (int i = 0; i < numberOfMeasurements; i++)
    {
        float angle = minAngleRad + i * angleResolutionRad;
        DDA::_2D::RayCastInfo info =
            DDA::_2D::castRay<CellState>(start, forward.rotate(angle), maxDistance, DDA_map, [](CellState c) { return c == CellState::Free; });
        
        //mtx.lock();
        if (info.hitSomething)
        {
            msg.ranges.push_back(info.distance);
        }
        else
        {
            msg.ranges.push_back(0);
        }
        //mtx.unlock();
    }

    return msg;
#endif
}
