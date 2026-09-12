#include "robot.hpp"
#include "helpers.hpp"
#include "macros.hpp"
#include "position_control.hpp"

void score_position_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, -100, 600, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -250, 600, 600);
}

void intake_position_macro()
{
    release_grip();

    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -610, 600, 800);
    position_control::move_absolute_blocking(position_control::MotorId::Lift, 0, 12000, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -20, 600, 800);
}