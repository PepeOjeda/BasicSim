#include <basic_sim/Logging.hpp>
#include <chrono>
#include <cmath>

namespace Profiling
{

    class ScopedStopwatch
    {
        using TimePoint = std::chrono::_V2::system_clock::time_point;

    public:
        ScopedStopwatch(const std::string& _name = "")
        {
            start = clock.now();
            name = _name;
        }

        ~ScopedStopwatch()
        {
            double seconds = ellapsed();
            BS_INFO("%s - Ellapsed: %fs", name.c_str(), seconds);
        }

        double ellapsed()
        {
            auto nanoseconds = (clock.now() - start).count();
            double seconds = nanoseconds / 10e9;
            return seconds;
        }

    private:
        std::chrono::system_clock clock;
        TimePoint start;
        std::string name;
    };

} // namespace Profiling