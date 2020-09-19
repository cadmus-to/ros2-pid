/**
 * @file config.cpp
 * @brief
 * @version 0.1
 * @date 2020-09-14
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "pid/config.hpp"

namespace pid
{

bool Config::is_valid() const
{
    if (this->windup_limit < 0)
    {
        throw std::runtime_error("windup_limit must be greater than 0.");
    }

    if (this->lower_limit >= this->upper_limit)
    {
        throw std::runtime_error("upper_limit must be greater than lower_limit.");
    }

    return true;
}

} // namespace pid
