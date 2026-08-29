#ifndef POSITION_CONTROL_HPP
#define POSITION_CONTROL_HPP

#include <cstdint>

namespace position_control {

enum class MotorId : std::uint8_t {
    Lift,
    Arm,
    Wrist
};

enum class Status : std::uint8_t {
    Pending,
    Running,
    Completed,
    TimedOut,
    Cancelled,
    Invalid
};

using CommandId = std::uint32_t;

void start();

// Maintained automatically by the position-control task every 10 ms.
// The wrist target is derived from the arm encoder position.
void update_wrist_parallel();
void set_arm_manual_input(std::int32_t input);

CommandId move_absolute(MotorId motor, double position, std::int32_t max_velocity_rpm,
                        std::uint32_t timeout_ms);
CommandId move_relative(MotorId motor, double delta, std::int32_t max_velocity_rpm,
                        std::uint32_t timeout_ms);
CommandId move_absolute_degrees(MotorId motor, double position_degrees,
                                std::int32_t max_velocity_rpm, std::uint32_t timeout_ms);
CommandId move_relative_degrees(MotorId motor, double delta_degrees,
                                std::int32_t max_velocity_rpm, std::uint32_t timeout_ms);

Status get_status(CommandId command);
bool cancel(CommandId command);

Status move_absolute_blocking(MotorId motor, double position, std::int32_t max_velocity_rpm,
                              std::uint32_t timeout_ms);
Status move_relative_blocking(MotorId motor, double delta, std::int32_t max_velocity_rpm,
                              std::uint32_t timeout_ms);
Status move_absolute_degrees_blocking(MotorId motor, double position_degrees,
                                      std::int32_t max_velocity_rpm, std::uint32_t timeout_ms);
Status move_relative_degrees_blocking(MotorId motor, double delta_degrees,
                                      std::int32_t max_velocity_rpm, std::uint32_t timeout_ms);

}

#endif