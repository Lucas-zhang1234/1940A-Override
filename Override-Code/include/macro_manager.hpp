enum class Macro {
    NONE,
    INTAKE_POSITION,
    SCORE_POSITION,
};

bool isMacroRunning();

void clearMacros();

bool tryAddMacroToQueue(Macro macro);

void macroTask(void* param);