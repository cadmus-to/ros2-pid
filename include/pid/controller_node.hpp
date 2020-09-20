/**
 * @file controller_node.hpp
 * @brief
 * @version 0.1
 * @date 2020-09-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef PID_CONTROLLER_NODE_HPP
#define PID_CONTROLLER_NODE_HPP

//////////////////////
// INCLUDES
//////////////////////

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "pid/config.hpp"
#include "pid/controller.hpp"

//////////////////////
// CLASSES
//////////////////////

namespace pid
{

class ControllerNode : public rclcpp::Node
{

  public:
    /**
     * @brief Construct a new controller node.
     *
     */
    ControllerNode();

    ~ControllerNode() noexcept = default;

    /**
     * @brief Update node.
     *
     */
    void update();

  private:
    bool enabled_ = true;
    Controller controller_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr plant_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;

    // Publishers.
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_effort_publisher_;

    /**
     * @brief Declare ros parameters.
     *
     */
    void declare_parameters();

    /**
     * @brief Load controller pid from ros parameters.
     *
     */
    void load_controller_pid();

    /**
     * @brief Load controller config from ros parameters.
     *
     */
    void load_controller_config();

    // Callbacks.

    void plant_state_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
};

} // namespace pid

#endif