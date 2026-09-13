#include "robot.hpp"
#include "position_control.hpp"

bool isHolding = true;

void release_grip()
{
    isHolding = false;
    position_control::move_relative_degrees_blocking(position_control::MotorId::Grip, 600, 12000, 600);
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