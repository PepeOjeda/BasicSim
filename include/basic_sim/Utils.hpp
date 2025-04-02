#pragma once
#include <random>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Utils
{
    static thread_local std::minstd_rand0 RNGengine;
    inline geometry_msgs::msg::Pose transformToPose(const geometry_msgs::msg::Transform& tf)
    {
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf.rotation;
        pose.position.x = tf.translation.x;
        pose.position.y = tf.translation.y;
        pose.position.z = tf.translation.z;
        return pose;
    }

    inline double randomFromGaussian(double mean, double stdev)
    {
        static thread_local std::normal_distribution<> dist{0, stdev};
        static thread_local double previousStdev = stdev;

        if (stdev != previousStdev)
        {
            dist = std::normal_distribution<>{0, stdev};
            previousStdev = stdev;
        }

        return mean + dist(RNGengine);
    }

    inline float uniformRandom(float min, float max)
    {
        static thread_local std::uniform_real_distribution<float> distribution{0.0, 0.999};
        return min + distribution(RNGengine) * (max - min);
    }

    // holds a long list of N(0,1) values, and returns them one at a time, scaled as requested.
    // obviously not as good as generating them on the fly, but it's not like we are doing cryptography here
    template <int Size>
    class PrecalculatedGaussian
    {
    public:
        PrecalculatedGaussian()
        {
            m_index = uniformRandom(0, Size);
            for (size_t i = 0; i < Size; i++)
                m_precalculatedTable[i] = Utils::randomFromGaussian(0, 1);
        }

        float nextValue(float mean, float stdev)
        {
            m_index = (m_index + 1) % Size;
            return mean + stdev * m_precalculatedTable[m_index];
        }

    private:
        uint16_t m_index;
        std::array<float, Size> m_precalculatedTable;
    };
}