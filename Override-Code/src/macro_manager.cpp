#include "macros.hpp"
#include "robot.hpp"
#include <queue>

enum class Macro {
    NONE,
    INTAKE_POSITION,
    FAST_INTAKE_POSITION,
    SCORE_POSITION,
    PICK_UP_PIN,
    SCORE_ONE_PIN,
    INTAKE_POSITION2
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

                case Macro::INTAKE_POSITION:
                    macroRunning = true;
                    intake_position_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::INTAKE_POSITION2:
                    macroRunning = true;
                    intake_position_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::FAST_INTAKE_POSITION:
                    macroRunning = true;
                    fast_intake_position_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::SCORE_POSITION:
                    macroRunning = true;
                    score_position_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::PICK_UP_PIN:
                    macroRunning = true;
                    pick_up_pin_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

                case Macro::SCORE_ONE_PIN:
                    macroRunning = true;
                    score_one_pin_macro();
                    macroRunning = false;
                    macroQueue.pop();
                    break;

            }
        }
        
        pros::delay(10);
    }
}