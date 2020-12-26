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

/**
 * @brief PID Controller object. For more info see: <https://en.wikipedia.org/wiki/PID_controller>.
 * By constructing the object and by changing the `PID` values, changes how strong the control
 * effort is while it is trying to reach its target (henceforth refferred to  as the `setpoint`).
 *
 * The `setpoint` determines the goal of the control system. The current state of the system is
 * present in `plant_state`. This will return a `control_effort`.
 *
 * By adding a `Config` it can be configured how the controller reaches its target. For more info,
 * refer to the `Config` structure.
 *
 * To update the controller use the `update()` function. This will automatically take account of the
 * deltaTime used in the calculations.
 *
 *
 * @todo Add the `PID` struct to the constructor. This object should be function without having to
 * build it as a node.
 */
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