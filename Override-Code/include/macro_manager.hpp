enum class Macro {
    NONE,
    GRAB_PIN,
    SCORE_POSITION,
};

bool isMacroRunning();

void clearMacros();

bool tryAddMacroToQueue(Macro macro);

void macroTask(void* param);