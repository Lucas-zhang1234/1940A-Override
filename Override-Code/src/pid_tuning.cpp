#include "robot.hpp"

void turn()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 1000);
}

void move()
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 1000);
}