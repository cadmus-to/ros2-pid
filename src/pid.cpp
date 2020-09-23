/**
 * @file pid.cpp
 * @brief
 * @version 0.1
 * @date 2020-09-14
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "jlb_pid/pid.hpp"

namespace jlbpid
{

bool PID::is_valid() const
{
    return (this->kp <= 0 && this->ki <= 0 && this->kd <= 0) ||
           (this->kp >= 0 && this->ki >= 0 && this->kd >= 0);
}

} // namespace jlbpid
