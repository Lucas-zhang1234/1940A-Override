enum class Macro {
    NONE,
    INTAKE_POSITION,
    INTAKE_POSITION2,
    FAST_INTAKE_POSITION,
    SCORE_POSITION,
    PICK_UP_PIN,
    SCORE_ONE_PIN,
};

bool isMacroRunning();

void clearMacros();

bool tryAddMacroToQueue(Macro macro);

void macroTask(void* param);