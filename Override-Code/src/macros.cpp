#include "pros/rtos.hpp"
#include "robot.hpp"
#include "helpers.hpp"
#include "macros.hpp"
#include "position_control.hpp"

void score_position_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -250, 600, 600);
}

void intake_position_macro()
{
    pros::Task release_task(release_grip, (void*)(intptr_t)1000, "Release Grip Task");
    position_control::move_relative_degrees(position_control::MotorId::Lift, 300, 600, 1300);
    pros::delay(250);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -610, 600, 800);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 195, 12000, 1800);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -20, 600, 800);
}

void intake_position2_macro()
{
    pros::Task release_task(release_grip, (void*)(intptr_t)1000, "Release Grip Task");
    position_control::move_relative_degrees(position_control::MotorId::Lift, 300, 600, 1300);
    pros::delay(250);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -610, 600, 800);
}

void fast_intake_position_macro()
{
    pros::Task release_task(release_grip, (void*)(intptr_t)400, "Release Grip Task");
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -610, 600, 800);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 195, 12000, 1800);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -20, 600, 800);
}

void pick_up_pin_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 0, 600, 600);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 600, 450);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -250, 12000, 500);
}

void score_one_pin_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -225, 12000, 1000);
    pros::Task release_slow_task(release_grip_slow, (void*)(intptr_t)900, "Release Grip Slow Task");
    pros::delay(300);
}

// void score_one_pin2_macro()
// {
//     position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -225, 12000, 1000);
//     pros::Task release_slow_task(release_grip_slow, (void*)(intptr_t)900, "Release Grip Slow Task");
//     pros::delay(300);
// }