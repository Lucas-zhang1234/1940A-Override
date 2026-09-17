#include "robot.hpp"
#include "position_control.hpp"

bool isHolding = true;

void release_grip(void* param)
{
    int ms = (int)(intptr_t)param;
    
    isHolding = false;
    Grip.move_voltage(6000);
    pros::delay(ms);
    isHolding = true;
}

void hold_grip()
{
    isHolding = true;
    Grip.move_voltage(-6000);
}

void hold_grip_task(void* param)
{
    while (true)
    {
        if (isHolding)
        {
            hold_grip();
        }
        pros::delay(20);
    }
}