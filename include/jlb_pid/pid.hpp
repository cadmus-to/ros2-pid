/**
 * @file pid.hpp
 * @brief Contains the PID data structure.
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

namespace jlbpid
{

/**
 * @brief Structure used to represent a PID controller.
 *
 */
struct PID
{
    double kp;
    double ki;
    double kd;

    /**
     * @brief Checks whether the values of the PID are valid or not.
     * If the PID values are invalid, it means the signs of `kp`, `ki` and `kd` are not equal.
     *
     * @return true The current state of the PID is valid
     * @return false The current state of the PID is invalid
     */
    bool is_valid() const;
};

} // namespace jlbpid

#endif