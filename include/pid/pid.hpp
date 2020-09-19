/**
 * @file pid.hpp
 * @brief
 * @version 0.1
 * @date 2020-09-14
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef PID_PID_HPP
#define PID_PID_HPP

//////////////////////
// STRUCTS
//////////////////////

namespace pid
{

struct PID
{
    double kp;
    double ki;
    double kd;

    bool is_valid() const;
};

} // namespace pid

#endif