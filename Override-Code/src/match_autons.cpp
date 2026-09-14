#include "position_control.hpp"
#include "robot.hpp"
#include "macro_manager.hpp"
#include "helpers.hpp"

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
    chassis.moveToPoint(-59, -13.27, 400, {.forwards=false});
    chassis.waitUntilDone();

    // toggle twice
    chassis.moveToPoint(-69.166, -13.27, 400, {.minSpeed=100});
    chassis.waitUntilDone();

    // move back
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 0, 600, 1500);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -600, 600, 1500);
    chassis.moveToPoint(-31.1, -13.27, 1300, {.forwards=false});
    chassis.waitUntilDone();
    chassis.turnToHeading(315, 500);
    chassis.waitUntilDone();

    // pick up pin and cup
    chassis.moveToPoint(-26, -22.5, 650, {.forwards=false});
    chassis.waitUntilDone();
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 600, 500);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -250, 600, 450);
    chassis.moveToPoint(-21.6, -25.6, 700, {.forwards=false});
    chassis.waitUntilDone();
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 800, 12000, 1500);
    chassis.turnToHeading(90, 700);
    chassis.waitUntilDone();
    chassis.moveToPoint(-49, -22.2, 1100, {.forwards=false, .minSpeed=20, .earlyExitRange=1});
    chassis.waitUntilDone();
    pros::Task release_task(release_grip, (void*)(intptr_t)400, "Release Grip Task");
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 1000, 12000, 1500);

    pros::delay(300);
    chassis.moveToPoint(-32, -28, 1000, {}, false);
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 1000, {}, false);

    chassis.moveToPoint(-50.2, -48.6, 1000, {.forwards=false});
    chassis.waitUntilDone();
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 0, 12000, 1500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 600, 500);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -250, 600, 450);
    position_control::move_absolute_blocking(position_control::MotorId::Lift, 900, 12000, 1000);
    chassis.turnToHeading(170, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-45.379, -24, 1000, {.forwards=false, .minSpeed=20, .earlyExitRange=1});
    chassis.waitUntilDone();
    pros::Task release_task_2(release_grip, (void*)(intptr_t)400, "Release Grip Task");
    pros::delay(280);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 1200, 12000, 1500);
    pros::delay(700);

    
}

void Q2_L_T4P()
{

}

void Q3_R_T2P()
{
    
}