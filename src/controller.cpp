/**
 * @file controller.cpp
 * @brief
 * @version 0.1
 * @date 2020-09-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "pid/controller.hpp"

namespace pid
{

Controller::Controller(std::size_t error_queue_size) : Controller(Config(), error_queue_size)
{
}

Controller::Controller(const Config &config, std::size_t error_queue_size)
    : error_queue_(std::deque<double>(error_queue_size)),
      error_derivitive_queue_(std::deque<double>(error_queue_size)),
      error_integral_queue_(std::deque<double>(error_queue_size)), config_(config),
      last_update_time_(this->clock_.now())
{
    if (error_queue_size < Controller::minimal_error_queue_size)
    {
        throw std::invalid_argument(
            "error_queue_size must be greater than or equal to minimal_error_queue_size.");
    }

    if (!config.is_valid())
    {
        throw std::runtime_error("Invalid config");
    }
}

Controller::Controller(Config &&config, std::size_t error_queue_size)
    : error_queue_(std::deque<double>(error_queue_size)),
      error_derivitive_queue_(std::deque<double>(error_queue_size)),
      error_integral_queue_(std::deque<double>(error_queue_size)), config_(std::move(config)),
      last_update_time_(this->clock_.now())
{
    if (error_queue_size < Controller::minimal_error_queue_size)
    {
        throw std::invalid_argument(
            "error_queue_size must be greater than or equal to minimal_error_queue_size.");
    }

    if (!config.is_valid())
    {
        throw std::runtime_error("Invalid config");
    }
}

Controller &Controller::operator=(Controller &&controller)
{
    if (this == &controller)
    {
        return *this;
    }

    this->error_queue_ = std::move(controller.error_queue_);
    this->config_ = std::move(controller.config_);

    return *this;
}

const PID &Controller::get_pid() const
{
    return this->pid_;
}

void Controller::set_pid(PID &&pid)
{
    if (!pid.is_valid())
    {
        throw std::runtime_error("Invalid pid");
    }

    this->pid_ = std::move(pid);
}

const Config &Controller::get_config() const
{
    return this->config_;
}

void Controller::set_config(Config &&config)
{
    if (!config.is_valid())
    {
        throw std::runtime_error("Invalid config");
    }

    this->config_ = std::move(config);
}

double Controller::get_control_effort()
{
    return this->control_effort_;
}

void Controller::set_setpoint(double setpoint)
{
    this->setpoint_ = setpoint;
    this->input_value_changed_ = true;
}

double Controller::get_setpoint()
{
    return this->setpoint_;
}

void Controller::set_plant_state(double plant_state)
{
    this->plant_state_ = plant_state;
    this->input_value_changed_ = true;
}

double Controller::get_plant_state()
{
    return this->plant_state_;
}

void Controller::update()
{
    this->current_update_time_ = this->clock_.now();
    this->delta_t_ = this->current_update_time_ - this->last_update_time_;

    if (this->delta_t_.seconds() == 0)
    {
        return;
    }

    this->update_error_();
    this->update_error_derivitive_();
    this->update_error_integral_();

    this->update_control_effort_();

    this->last_update_time_ = this->current_update_time_;
    this->input_value_changed_ = false;
}

void Controller::update_error_()
{
    this->last_error_ = this->error_;

    this->error_ = this->setpoint_ - this->plant_state_;

    this->error_queue_.push_back(this->error_);
    this->error_queue_.pop_front();
}

void Controller::update_error_derivitive_()
{
    this->error_derivitive_queue_.push_back((this->error_ - this->last_error_) /
                                            this->delta_t_.seconds());

    this->error_derivitive_queue_.pop_front();

    this->error_derivitive_ = std::accumulate(this->error_derivitive_queue_.begin(),
                                              this->error_derivitive_queue_.end(), 0.0) /
                              this->error_derivitive_queue_.size();
}

void Controller::update_error_integral_()
{
    this->error_integral_queue_.push_back(((this->last_error_ + this->error_) / 2) *
                                          this->delta_t_.seconds());

    this->error_integral_queue_.pop_front();

    this->error_integral_ = std::accumulate(this->error_integral_queue_.begin(),
                                            this->error_integral_queue_.end(), 0.0) /
                            this->error_integral_queue_.size();
}

void Controller::update_control_effort_()
{
    this->control_effort_ = (this->error_ * this->pid_.kp) +
                            (this->error_integral_ * this->pid_.ki) +
                            (this->error_derivitive_ * this->pid_.kd);

    if (this->control_effort_ > this->config_.upper_limit)
    {
        this->control_effort_ = this->config_.upper_limit;
    }
    else if (this->control_effort_ < this->config_.lower_limit)
    {
        this->control_effort_ = this->config_.lower_limit;
    }
}

} // namespace pid
