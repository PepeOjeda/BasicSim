#include "basic_sim/Logging.hpp"
#include "basic_sim/Utils.hpp"
#include <basic_sim/LaserScanner.hpp>

static thread_local Utils::PrecalculatedGaussian<2500> randomNumbers;

LaserScanner::LaserScanner(const LaserSensorDescription& desc, const Map* map)
    : description(desc)
{
    while (description.minAngleRad > description.maxAngleRad)
        description.maxAngleRad += 2 * M_PI;

    DDA_map = map->asDDAMap();
    DDA::loggingEnabled = false;
}

sensor_msgs::msg::LaserScan LaserScanner::Scan(const tf2::Vector3& position, const tf2::Vector3& forwardDirectionTF)
{
    int numberOfMeasurements = (description.maxAngleRad - description.minAngleRad) / description.angleResolutionRad;

    sensor_msgs::msg::LaserScan msg;
    msg.angle_min = description.minAngleRad;
    msg.angle_max = description.maxAngleRad;
    msg.angle_increment = description.angleResolutionRad;
    msg.time_increment = 0;
    msg.range_min = description.minDistance;
    msg.range_max = description.maxDistance;
    msg.ranges.resize(numberOfMeasurements, 0);

    DDA::Vector2 start(position.x(), position.y());
    DDA::Vector2 forward(forwardDirectionTF.x(), forwardDirectionTF.y());

    bool invalidReading = false;

#pragma omp parallel for
    for (int i = 0; i < numberOfMeasurements; i++)
    {
        float angle = description.minAngleRad + i * description.angleResolutionRad;
        DDA::_2D::RayCastInfo info =
            DDA::_2D::castRay<CellState>(start, forward.rotate(angle), description.maxDistance, DDA_map, [](CellState c)
                                         {
                                             return c == CellState::Free;
                                         });

        if (info.invalid())
            invalidReading = true;

        if (info.hitSomething)
            msg.ranges[i] = std::clamp(info.distance + randomNumbers.nextValue(0, description.noiseStdDev), description.minDistance, description.maxDistance);
        else
            msg.ranges[i] = description.maxDistance + 1; // invalid value, gets interpreted as a miss
    }

    if (invalidReading)
        BS_ERROR("At least one invalid reading when simulating laser scanner!");

    return msg;
}
