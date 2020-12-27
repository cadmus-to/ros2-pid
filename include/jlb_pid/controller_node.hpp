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

#include "jlb_pid/config.hpp"
#include "jlb_pid/controller.hpp"

//////////////////////
// CLASSES
//////////////////////

namespace jlbpid
{

/**
 * @brief Wraps around the `jlbpid::Controller` and makes the PID functionality accessible without
 * having to type any code. Also serves as great class to see how the `jlbpid::Controller` object
 * works.
 *
 */
class ControllerNode : public rclcpp::Node
{

  public:
    inline static constexpr double DEFAULT_UPDATE_RATE_HZ =
        1000; // Default update rate for the node, in `Hz`

    /**
     * @brief Construct a new controller node.
     *
     */
    ControllerNode();

    ~ControllerNode() noexcept = default;

    /**
     * @brief Get the requested update rate of the node.
     *
     * @return const std_msgs::msg::Float64& update rate in `Hz`
     */
    double get_update_rate() const;

    /**
     * @brief Update node.
     *
     */
    void update();

  private:
    bool enabled_ = true;
    Controller controller_;
    double update_rate;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr plant_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;

    // Publishers.
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_effort_publisher_;

    // Methods.
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

} // namespace jlbpid

#endif