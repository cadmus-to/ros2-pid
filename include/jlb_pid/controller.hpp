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
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "jlb_pid/config.hpp"
#include "jlb_pid/pid.hpp"

//////////////////////
// CLASSES
//////////////////////

namespace jlbpid
{

class Controller
{
  public:
    /**
     * @brief Construct a new PID controller object.
     */
    Controller();

    /**
     * @brief Construct a new PID controller object.
     *
     * @param config
     */
    Controller(const Config &config);

    /**
     * @brief Construct a new PID controller controller object.
     *
     * @param config
     */
    Controller(Config &&config);

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
    PID pid_;
    Config config_;

    rclcpp::Clock clock_;
    rclcpp::Time last_update_time_;
    rclcpp::Time current_update_time_;
    rclcpp::Duration delta_t_ = rclcpp::Duration(0, 0);

    double setpoint_;
    double plant_state_;

    double error_;
    double last_error_;
    double error_derivitive_;
    double error_integral_;

    double control_effort_;

    /**
     * @brief Update the error integral and error integral queue.
     *
     */
    void update_error_integral_();

    /**
     * @brief Update the error derivitive and error derivitive queue.
     *
     */
    void update_error_derivitive_();

    /**
     * @brief Update control effort based on the error values.
     *
     */
    void update_control_effort_();
};

} // namespace jlbpid

#endif