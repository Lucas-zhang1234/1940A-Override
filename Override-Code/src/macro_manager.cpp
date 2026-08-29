#include "macros.hpp"
#include "robot.hpp"
#include <queue>

enum class Macro {
    NONE,
    GRAB_PIN,
};

std::queue<Macro> macroQueue;

bool tryAddMacroToQueue(Macro macro)
{
    macroQueue.push(macro);
    if (macroQueue.size() == 1) return true;
    return false;
}

void macroTask(void* param) {

    while (true) {
        if (macroQueue.size() > 0)
        {
            switch (macroQueue.front()) 
            {
                case Macro::NONE:
                    break;

                case Macro::GRAB_PIN:
                    grab_pin_macro();
                    macroQueue.pop();
                    break;
            }
        }
        

        pros::delay(10);
    }
}