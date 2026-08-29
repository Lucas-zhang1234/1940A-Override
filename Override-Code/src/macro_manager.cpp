#include "macros.hpp"
#include "robot.hpp"
#include <queue>

enum class Macro {
    NONE,
    GRAB_PIN,
    SCORE_POSITION,
    ONE_PIN,
    TWO_PIN,
};

std::queue<Macro> macroQueue;

bool macroRunning = false;

bool tryAddMacroToQueue(Macro macro)
{
    macroQueue.push(macro);
    if (macroQueue.size() == 1) return true;
    return false;
}

bool isMacroRunning()
{
    return macroRunning;
}

void clearMacros()
{
    while (!macroQueue.empty()) {
        macroQueue.pop();
    }
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
                    macroRunning = true;
                    grab_pin_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::SCORE_POSITION:
                    macroRunning = true;
                    score_position_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::ONE_PIN:
                    macroRunning = true;
                    one_pin_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::TWO_PIN:
                    macroRunning = true;
                    two_pin_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;
            }
        }
        

        pros::delay(10);
    }
}