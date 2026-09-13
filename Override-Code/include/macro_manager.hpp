enum class Macro {
    NONE,
    INTAKE_POSITION,
    FAST_INTAKE_POSITION,
    SCORE_POSITION,
    PICK_UP_PIN,
};

bool isMacroRunning();

void clearMacros();

bool tryAddMacroToQueue(Macro macro);

void macroTask(void* param);