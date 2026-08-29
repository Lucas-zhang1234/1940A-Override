#include "robot.hpp"

void close_claw()
{
    Fingers.extend();
}

void open_claw()
{
    Fingers.retract();
}
