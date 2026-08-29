#include "robot.hpp"
#include "helpers.hpp"
#include "macros.hpp"
#include "position_control.hpp"

void grab_pin_macro()
{
    // Step 1: Move intake backwards
    Intake.move_voltage(-6000);
    pros::delay(250);

    // Step 2: Stop intake
    Intake.brake();

    // Step 3: Move the claw into position
    score_position_macro();

    // Step 4: Grab the pin with the claw
    close_claw();

    // Step 5: Lift up pin slightly, and parallel to ground

}

void score_position_macro()
{
    position_control::move_absolute_degrees(position_control::MotorId::Wrist, 0, 400, 2000);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 10, 600, 2000);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 0, 200, 2000);
}

void one_pin_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, -1028, 600, 2000);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -87.2, 600, 2000);
    // position_control::move_absolute_degrees_blocking(position_control::MotorId::Wrist, 100, 600, 2000);
    Wrist.move_absolute(250, 600);
    pros::delay(500);
}

void two_pin_macro()
{
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 146.4, 600, 2000);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, 620.8, 600, 2000);
    Wrist.move_absolute(-18, 600);
    pros::delay(700);
}