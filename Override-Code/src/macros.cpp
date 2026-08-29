#include "robot.hpp"
#include "helpers.hpp"
#include "macros.hpp"

void grab_pin_macro()
{
    // Step 1: Move intake backwards
    Intake.move_voltage(-12000);
    pros::delay(300);

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
    Lift.move_absolute(0, 12000);
    Arm.move_absolute(-10, 12000);
}