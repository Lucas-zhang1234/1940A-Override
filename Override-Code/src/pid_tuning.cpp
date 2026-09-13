#include "pros/screen.h"
#include "robot.hpp"

void turn()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000);
    pros::delay(2400);
}

void move()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 48, 3000);
}

void outputThetaTask(void* param)
{
    int i = 0;
    while (true)
    {
        pros::screen::print(pros::E_TEXT_SMALL, i, "Time: %d ms, IMU: %.2f\n", i * 100, IMU.get_heading());
        pros::delay(100);
        i++;
    }
}

void outputPosTask(void* param)
{
    int i = 0;
    while (true)
    {
        pros::screen::print(pros::E_TEXT_SMALL, i, "%d ms, H: (%.2f), xy: (%.2f, %.2f)\n", i * 200, chassis.getPose().theta, chassis.getPose().x, chassis.getPose().y);
        pros::delay(200);
    }
}