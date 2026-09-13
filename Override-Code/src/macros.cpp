#include "robot.hpp"
#include "helpers.hpp"
#include "macros.hpp"
#include "position_control.hpp"

void score_position_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 0, 600, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -250, 600, 600);
}

void intake_position_macro()
{
    release_grip();
    position_control::move_relative_degrees(position_control::MotorId::Lift, 800, 600, 1300);
    pros::delay(350);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -610, 600, 800);
    position_control::move_absolute_blocking(position_control::MotorId::Lift, 0, 12000, 2100);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -4, 600, 800);
}

void fast_intake_position_macro()
{
    release_grip();
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -610, 600, 800);
    position_control::move_absolute_blocking(position_control::MotorId::Lift, 0, 12000, 2100);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -4, 600, 800);
}

void pick_up_pin_macro()
{
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 0, 600, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -50, 600, 1500);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 200, 600, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -250, 600, 1500);
}