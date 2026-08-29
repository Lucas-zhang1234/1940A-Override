enum class Macro {
    NONE,
    GRAB_PIN,
    SCORE_POSITION,
    ONE_PIN,
    TWO_PIN
};

bool isMacroRunning();

void clearMacros();

bool tryAddMacroToQueue(Macro macro);

void macroTask(void* param);