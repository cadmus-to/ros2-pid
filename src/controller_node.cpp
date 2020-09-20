/**
 * @file controller.cpp
 * @brief
 * @version 0.1
 * @date 2020-09-12
 *
 * @copyright Copyright (c) 2020
 *
 */

//////////////////////
// INCLUDES
//////////////////////

#include <cstdio>

#include "pid/controller_node.hpp"

//////////////////////
// METHODS
//////////////////////

namespace pid
{

ControllerNode::ControllerNode() : rclcpp::Node("controller_node")
{
    this->declare_parameters();

    this->load_controller_pid();
    this->load_controller_config();

    // Subscribers / Publishers.
    std::string topic;
    this->get_parameter_or<std::string>("plant_topic", topic, "state");
    this->plant_state_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        topic, 1, std::bind(&ControllerNode::plant_state_callback, this, std::placeholders::_1));

    this->get_parameter_or<std::string>("setpoint_topic", topic, "setpoint");
    this->setpoint_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        topic, 1, std::bind(&ControllerNode::setpoint_callback, this, std::placeholders::_1));

    this->get_parameter_or<std::string>("enable_topic", topic, "enable");
    this->enable_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        topic, 1, std::bind(&ControllerNode::enable_callback, this, std::placeholders::_1));

    this->get_parameter_or<std::string>("controller_topic", topic, "control_effort");
    this->control_effort_publisher_ = this->create_publisher<std_msgs::msg::Float64>(topic, 1);
}

void ControllerNode::update()
{
    if (!this->enabled_)
    {
        return;
    }

    this->controller_.update();

    std_msgs::msg::Float64 control_effort_msg;
    control_effort_msg.data = this->controller_.get_control_effort();
    this->control_effort_publisher_->publish(control_effort_msg);
}

void ControllerNode::declare_parameters()
{
    // Subscribers.
    this->declare_parameter("plant_topic");
    this->declare_parameter("setpoint_topic");
    this->declare_parameter("enable_topic");

    // Publishers.
    this->declare_parameter("controller_topic");

    // PID.
    this->declare_parameter("ki");
    this->declare_parameter("kp");
    this->declare_parameter("kd");

    // Controller config.
    this->declare_parameter("upper_limit");
    this->declare_parameter("lower_limit");

    this->declare_parameter("windup_limit");
}

void ControllerNode::load_controller_pid()
{
    PID pid;
    this->get_parameter<double>("ki", pid.ki);
    this->get_parameter<double>("kp", pid.kp);
    this->get_parameter<double>("kd", pid.kd);

    this->controller_.set_pid(std::move(pid));
}

void ControllerNode::load_controller_config()
{
    Config config;

    this->get_parameter<double>("upper_limit", config.upper_limit);
    this->get_parameter<double>("lower_limit", config.lower_limit);

    this->get_parameter<double>("windup_limit", config.windup_limit);

    this->controller_.set_config(std::move(config));
}

void ControllerNode::plant_state_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    this->controller_.set_plant_state(msg->data);
}

void ControllerNode::setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    this->controller_.set_setpoint(msg->data);
}

void ControllerNode::enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    this->enabled_ = msg->data;
}

} // namespace pid

//////////////////////
// FUNCTIONS
//////////////////////

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto controllerNode = std::make_shared<pid::ControllerNode>();

    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
        controllerNode->update();
        rclcpp::spin_some(controllerNode);
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
