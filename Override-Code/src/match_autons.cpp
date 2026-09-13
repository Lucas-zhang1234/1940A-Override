#include "position_control.hpp"
#include "robot.hpp"
#include "macro_manager.hpp"

void Q1_L_T2P()
{

}

void Q1_R_T4P()
{
    chassis.setPose(-61.172, -14.475 , 297);

    tryAddMacroToQueue(Macro::SCORE_POSITION);
    pros::delay(200);

    // stack one pin in the goal
    chassis.moveToPoint(-51.8, -18.8, 700, {.forwards=false, .minSpeed=40, .earlyExitRange=2});
    chassis.waitUntilDone();
    tryAddMacroToQueue(Macro::FAST_INTAKE_POSITION);
    pros::delay(270);

    // toggle once
    chassis.moveToPoint(-69.166, -12.136, 750, {.minSpeed=100});
    chassis.waitUntilDone();
    
    // move back
    chassis.moveToPoint(-60, -13.27, 400, {.forwards=false});
    chassis.waitUntilDone();

    // toggle twice
    chassis.moveToPoint(-69.166, -13.27, 400, {.minSpeed=100});
    chassis.waitUntilDone();

    // move back
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 0, 600, 1500);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -600, 600, 1500);
    chassis.moveToPoint(-31.1, -13.27, 1300, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(315, 1200);
    chassis.waitUntilDone();

    // pick up pin and cup
    chassis.moveToPoint(-25.297, -23, 1000, {.forwards=false});
    chassis.waitUntilDone();
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 600, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, -250, 600, 1500);
}

void Q2_L_T4P()
{

}

void Q3_R_T2P()
{
    
}