#pragma once
#include <rclcpp/logging.hpp>

#define BS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("basic_sim"), __VA_ARGS__);
#define BS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("basic_sim"), __VA_ARGS__);
