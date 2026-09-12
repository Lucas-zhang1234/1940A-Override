#include "robot.hpp"

void turn()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000);
    pros::delay(2400);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "IMU: %.2f deg", IMU.get_heading());
}

void move()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 1000);
}

void outputThetaTask(void* param)
{
    int i = 0;
    while (true)
    {
        pros::screen::print(pros::E_TEXT_SMALL, i, "Time: %d ms, IMU: %.2f\n", i * 200, IMU.get_heading());
        pros::delay(200);
        i++;
    }
}