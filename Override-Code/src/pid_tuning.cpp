#include "robot.hpp"

void turn()
{
    Chassis.setPose(0, 0, 0);
    Chassis.turnToHeading(90, 1000);
}

void move()
{
    
}