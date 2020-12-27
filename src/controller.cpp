/**
 * @file controller.cpp
 * @brief
 * @version 0.1
 * @date 2020-09-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "jlb_pid/controller.hpp"

namespace jlbpid
{

Controller::Controller()
{
}

Controller::Controller(const Config &config)
    : config_(config), last_update_time_(this->clock_.now())
{
    if (!config.is_valid())
    {
        throw std::runtime_error("Invalid config");
    }
}

Controller::Controller(Config &&config)
    : config_(std::move(config)), last_update_time_(this->clock_.now())
{
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

double Controller::get_control_effort() const
{
    return this->control_effort_;
}

void Controller::set_setpoint(double setpoint)
{
    if (this->setpoint_ == setpoint)
    {
        return;
    }

    this->setpoint_ = setpoint;
}

double Controller::get_setpoint() const
{
    return this->setpoint_;
}

void Controller::set_plant_state(double plant_state)
{
    this->plant_state_ = plant_state;
}

double Controller::get_plant_state() const
{
    return this->plant_state_;
}

void Controller::update()
{
    this->current_update_time_ = this->clock_.now();
    this->delta_t_ = this->current_update_time_ - this->last_update_time_;

    if (this->delta_t_.seconds() <= 0.0)
    {
        return;
    }

    this->error_ = this->setpoint_ - this->plant_state_;

    this->update_error_integral_();
    this->update_error_derivitive_();

    this->update_control_effort_();

    this->last_error_ = this->error_;
    this->last_update_time_ = this->current_update_time_;
}

void Controller::update_error_integral_()
{
    this->error_integral_ += ((this->last_error_ + this->error_) / 2) * this->delta_t_.seconds();

    if (this->error_integral_ > this->config_.windup_limit)
    {
        this->error_integral_ = this->config_.windup_limit;
    }
    else if (this->error_integral_ < -this->config_.windup_limit)
    {
        this->error_integral_ = -this->config_.windup_limit;
    }
}

void Controller::update_error_derivitive_()
{
    this->error_derivitive_ = (this->error_ - this->last_error_) / this->delta_t_.seconds();
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

} // namespace jlbpid
