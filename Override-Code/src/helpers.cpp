#include "robot.hpp"
#include "position_control.hpp"

bool isHolding = false;

void release_grip()
{
    position_control::move_relative_degrees_blocking(position_control::MotorId::Grip, 600, 12000, 600);
}

void hold_grip()
{
    Grip.move_voltage(-6000);
}