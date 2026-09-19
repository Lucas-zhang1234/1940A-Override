#include "robot.hpp"
#include "position_control.hpp"
#include "helpers.hpp"
#include "macro_manager.hpp";

void skills()
{
    chassis.setPose(-61.172, -14.475 , 297);

    position_control::move_absolute_degrees(position_control::MotorId::Arm, -235, 12000, 1000);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 160, 600, 1500);

    // stack one pin in the goal
    chassis.moveToPoint(-51.5, -19.5, 900, {.forwards=false});
    chassis.waitUntilDone();
    pros::Task release_task(release_grip_slow, (void*)(intptr_t)800, "Release Grip Task");
    pros::delay(450);

    // toggle once
    chassis.moveToPoint(-69.166, -12.136, 750, {.minSpeed=100});
    chassis.waitUntilDone();

    position_control::move_absolute_degrees(position_control::MotorId::Arm, -250, 12000, 1000);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 400, 600, 1500);
    
    // move back
    chassis.moveToPoint(-59, -13.27, 400, {.forwards=false});
    chassis.waitUntilDone();

    // toggle twice
    chassis.moveToPoint(-69.166, -13.27, 400, {.minSpeed=100});
    chassis.waitUntilDone();

    // move back
    chassis.moveToPoint(-34, -13.27, 1300, {.forwards=false});
    chassis.turnToHeading(315, 500);
    chassis.waitUntilDone();

    // pick up pin and cup
    chassis.moveToPoint(-28, -24, 650, {.forwards=false});
    chassis.waitUntilDone();
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 0, 600, 500);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 600, 300);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -250, 600, 450);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 450, 12000, 1500);
    chassis.turnToHeading(90, 700);
    chassis.waitUntilDone();
    chassis.moveToPoint(-51.8, -24.9, 1100, {.forwards=false});
    chassis.waitUntilDone();
    pros::Task release_task_2(release_grip, (void*)(intptr_t)400, "Release Grip Task");
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 600, 12000, 1500);

    pros::delay(300);
    chassis.moveToPoint(-32, -20, 1000, {}, false);
    chassis.waitUntilDone();
    chassis.turnToHeading(33, 1000, {}, false);

    chassis.moveToPoint(-50, -49, 1000, {.forwards=false});
    chassis.waitUntilDone();
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 0, 12000, 750);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 600, 300);
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -250, 600, 400);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 750, 12000, 1400);
    chassis.turnToHeading(160, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-51.3, -25.2, 1000, {.forwards=false, .minSpeed=20, .earlyExitRange=1});
    chassis.waitUntilDone();
    pros::Task release_task_3(release_grip, (void*)(intptr_t)400, "Release Grip Task");
    pros::delay(220);
    position_control::move_absolute_degrees(position_control::MotorId::Lift,980, 12000, 1500);
    pros::delay(100);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 165, 12000, 1000);
    chassis.moveToPoint(-62, -56.5, 1000);
    chassis.waitUntilDone();
    
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -10, 12000, 1000);
    chassis.turnToHeading(270, 1000);

    for (int i = 0; i < 2; i++)
    {
        chassis.moveToPoint(-73 - i * 4, -52 + i * 1, 1300);
        chassis.waitUntilDone();
        Intake.move_voltage(-12000);
        pros::delay(2400);
        for (int j = 0; j < 1; j++)
        {
            chassis.moveToPoint(-58, -53.2, 1000, {.forwards=false, .minSpeed=30});
            chassis.moveToPoint(-69, -53.2, 1000,{.minSpeed=30});
        }
        chassis.moveToPoint(-53 - i * 5, -49, 1000, {.forwards=false});
        position_control::move_absolute_degrees_blocking(position_control::MotorId::Arm, 0, 12000, 600);
        position_control::move_absolute_degrees(position_control::MotorId::Arm, -250, 12000, 1000);
        position_control::move_absolute_degrees(position_control::MotorId::Lift, 1050 + 300 * i, 12000, 1800);
        chassis.turnToHeading(180, 1000);
        chassis.moveToPoint(-62.5 - i * 4, -16, 1500, {.forwards=false});

        chassis.waitUntilDone();
        pros::Task release_task_4(release_grip, (void*)(intptr_t)400, "Release Grip Task");
        pros::delay(220);
        position_control::move_absolute_degrees(position_control::MotorId::Lift,1250 + 300 * i, 12000, 1000);
        pros::delay(100);
        position_control::move_absolute_degrees(position_control::MotorId::Lift, 165, 12000, 1500);

        chassis.moveToPoint(-56, -47, 1300);
        chassis.waitUntilDone();
        position_control::move_absolute_degrees(position_control::MotorId::Arm, -10, 600, 800);
        chassis.turnToHeading(270, 1000);
        // maybe use distance sensors
    }

    chassis.moveToPoint(-79, -50, 1000);
    chassis.waitUntilDone();
    Intake.move_voltage(-12000);

    pros::delay(2400);

    chassis.moveToPoint(-66, -52, 1000, {.forwards=false, .minSpeed=30});

    chassis.turnToHeading(225, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-18, 8, 2000, {.forwards=false});
    position_control::move_absolute_degrees(position_control::MotorId::Arm, -235, 12000, 1000);
    position_control::move_absolute_degrees(position_control::MotorId::Lift, 370, 12000, 1000);
    chassis.waitUntilDone();
    tryAddMacroToQueue(Macro::SCORE_ONE_PIN);
    pros::delay(400);
    position_control::move_absolute_degrees_blocking(position_control::MotorId::Lift, 600, 12000, 1000);
}