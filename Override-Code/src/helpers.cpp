#include "robot.hpp"

void close_claw()
{
    Claw_Grip.extend();
}

void open_claw()
{
    Claw_Grip.retract();
}
