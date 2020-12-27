/**
 * @file config.hpp
 * @brief
 * @version 0.1
 * @date 2020-09-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef PID_CONFIG_HPP
#define PID_CONFIG_HPP

//////////////////////
// INCLUDE
//////////////////////

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "jlb_pid/pid.hpp"

//////////////////////
// STRUCTS
//////////////////////

namespace jlbpid
{

struct Config
{
    /**
     * @brief Upper limit. Must be greater than lower_limit.
     */
    double upper_limit = std::numeric_limits<double>::max();

    /**
     * @brief Lower limit. Must be less than upper_limit.
     */
    double lower_limit = std::numeric_limits<double>::lowest();

    /**
     * @brief Windup limit. Must be greater than 0.
     */
    double windup_limit = std::numeric_limits<double>::max();

    /**
     * @brief Check if all config values are valid.
     *
     * @return true
     * @return false
     *
     * @throw std::runtime_error Current `windup_limit` is <0, must be >= 0
     * @throw std::runtime_error `lower_limit` >= `upper_limit`, `upper_limit` *must* be higher than
     * `lower_limit`
     */
    bool is_valid() const;
};

} // namespace jlbpid

#endif
