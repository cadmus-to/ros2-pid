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
 * @brief PID Controller class.
 * When given a target (henceforth referred to as `setpoint`) the system will return a
 * `control_effort` based on how much the current state (henceforth referred to as `plant_state`)
 * deviates from the `setpioint`.
 *
 * The strength of the `control_effort` changes based on values used for the `PID`.
 * How fast a `setpoint` can be reached also depends on the provided `Config`.
 *
 * More info: <https://en.wikipedia.org/wiki/PID_controller>
 *
 * @example ControllerNode controller_node.hpp controller_node.cpp
 *
 */
class Controller
{
  public:
    /**
     * @brief Construct a new Controller object
     *
     * @param pid The values for the PID system, defaults to {kp: 1, ki: 0, kd: 0}
     * @param config The configuration for the PID system, uses all default values for the `Config`.
     *
     * @throw std::runtime_error PID is deemed invalid, see `PID::is_vallid`
     * @throw std::runtime_error Config is deemed invalid, see `Config::is_vallid`.
     */
    Controller(PID &&pid = {1, 0, 0}, Config &&config = {});

    ~Controller() noexcept = default;

    Controller &operator=(Controller &&controller);

    /**
     * @brief Get the pid object
     *
     * @return const PID& An immutable reference to the current `PID` configuration
     */
    const PID &get_pid() const;

    /**
     * @brief Set the pid object
     *
     * @param pid new *valid* `PID` config
     *
     * @throw std::runtime_error The provided values for PID are deemed invalid, see
     * `PID::is_vallid`.
     */
    void set_pid(PID &&pid);

    /**
     * @brief Get the config object
     *
     * @return const Config& An immutable reference to the current `Config`
     */
    const Config &get_config() const;

    /**
     * @brief Set the config object
     *
     * @param config new *valid* `Config`
     *
     * @throw std::runtime_error The provided values for the config are deemed invalid, see
     * `Config::is_vallid`
     */
    void set_config(Config &&config);

    /**
     * @brief Set the setpoint object
     *
     * @param setpoint The new target the system is trying to reach
     */
    void set_setpoint(double setpoint);

    /**
     * @brief Get the setpoint object
     *
     * @return double
     */
    double get_setpoint() const;

    /**
     * @brief Set the plant state of the system (i.e. the current value of the system it is trying
     * to control).
     *
     * @param plant_state The value the system currently has
     *
     * @note Changing the plant_state does *not* update the system, use `Controller::update` for
     * that.
     */
    void set_plant_state(double plant_state);

    /**
     * @brief Get the last registered state of the system it is trying to control
     *
     * @return double
     */
    double get_plant_state() const;

    /**
     * @brief Get the control effort from the system.
     *
     * @return double
     *
     * @note The values of the control effort are changed after `Controller:update` is called
     */
    double get_control_effort() const;

    /**
     * @brief Updates controller.
     *
     * @pre `plant_state` has been set using `Controller::set_plant_state`.
     * @post `control_effort` is updated and can be accessed using `Controller::get_control_effort`.
     */
    void update();

    /**
     * @brief Updates the controller using a given plant state
     *
     * @param plant_state The value the system currently has
     * @return double The control effort
     *
     * @note This function merges the functionality of `set_plant_state`, `update` and
     * `get_control_effort`.
     */
    double update(double plant_state);

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