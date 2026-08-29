#include "robot.hpp"

void tune()
{
    Chassis.setPose(0, 0, 0);
    Chassis.turnToHeading(90, 1000);
}