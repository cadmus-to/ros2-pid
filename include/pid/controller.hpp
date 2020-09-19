/**
 * @file controller.hpp
 * @brief
 * @version 0.1
 * @date 2020-09-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

//////////////////////
// INCLUDES
//////////////////////

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "pid/config.hpp"
#include "pid/pid.hpp"

//////////////////////
// CLASSES
//////////////////////

namespace pid
{

class Controller
{
  public:
    // Default queue sizes.
    static constexpr std::size_t default_error_queue_size = 5;
    static constexpr std::size_t minimal_error_queue_size = 2;

    static_assert(
        Controller::default_error_queue_size >= Controller::minimal_error_queue_size,
        "default_error_queue_size must be greater than or equal to minimal_error_queue_size.");

    /**
     * @brief Construct a new PID controller object.
     *
     * @param error_queue_size
     */
    Controller(std::size_t error_queue_size = Controller::default_error_queue_size);

    /**
     * @brief Construct a new PID controller object.
     *
     * @param config
     * @param error_queue_size
     */
    Controller(const Config &config,
               std::size_t error_queue_size = Controller::default_error_queue_size);

    /**
     * @brief Construct a new PID controller controller object.
     *
     * @param config
     * @param error_queue_size
     */
    Controller(Config &&config,
               std::size_t error_queue_size = Controller::default_error_queue_size);

    ~Controller() noexcept = default;

    Controller &operator=(Controller &&controller);

    const PID &get_pid() const;

    void set_pid(PID &&pid);

    const Config &get_config() const;

    void set_config(Config &&config);

    void set_setpoint(double setpoint);

    double get_setpoint();

    void set_plant_state(double plant_state);

    double get_plant_state();

    double get_control_effort();

    /**
     * @brief Update controller.
     *
     */
    void update();

  private:
    std::deque<double> error_queue_;
    std::deque<double> error_derivitive_queue_;
    std::deque<double> error_integral_queue_;

    PID pid_;
    Config config_;

    rclcpp::Clock clock_;
    rclcpp::Time last_update_time_;
    rclcpp::Time current_update_time_;
    rclcpp::Duration delta_t_ = rclcpp::Duration(0, 0);

    double setpoint_ = 0.0;
    double plant_state_ = 0.0;
    bool input_value_changed_ = false;
    double error_ = 0.0;
    double last_error_ = 0.0;
    double error_derivitive_ = 0.0;
    double error_integral_ = 0.0;

    double control_effort_ = 0.0;

    /**
     * @brief Update the error and error queue.
     *
     */
    void update_error_();

    /**
     * @brief Update the error derivitive and error derivitive queue.
     *
     */
    void update_error_derivitive_();

    /**
     * @brief Update the error integral and error integral queue.
     *
     */
    void update_error_integral_();

    /**
     * @brief Update control effort based on the error values.
     *
     */
    void update_control_effort_();
};

} // namespace pid

#endif