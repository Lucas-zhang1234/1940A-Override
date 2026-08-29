#include "claw_leveling.hpp"

#include "pros/rtos.hpp"
#include "robot.hpp"

#include <cmath>
#include <cstdint>

namespace claw_leveling {
namespace {

constexpr double ArmDirection = 1.0;
constexpr double WristDirection = 1.0;
constexpr double ArmMotorToJointRatio = 5.0; // in motor encoder degrees per joint degree
constexpr double WristMotorToJointRatio = 3.5; // in motor encoder degrees per joint degree
constexpr double WristMotorToArmMotorRatio =
    WristMotorToJointRatio / ArmMotorToJointRatio;
constexpr double ArmFlipPosition = 750.0;
constexpr double FlipMotorDegrees = 180.0 * WristMotorToJointRatio;
constexpr std::int32_t WristVelocity = 600;
constexpr std::uint32_t UpdatePeriodMs = 20;

pros::Task* leveling_task = nullptr;
bool running = false;

void update() {
    bool has_previous_arm_position = false;
    double previous_arm_position = 0.0;
    double wrist_target = 0.0;
    bool is_flipped = false;
    while (true) {
        if (!running) {
            has_previous_arm_position = false;
            pros::delay(UpdatePeriodMs);
            continue;
        }

        const double arm_position = Arm.get_position();
        if (!has_previous_arm_position) {
            previous_arm_position = arm_position;
            wrist_target = Wrist.get_position();
            has_previous_arm_position = true;
            pros::delay(UpdatePeriodMs);
            continue;
        }

        const double arm_delta = arm_position - previous_arm_position;
        previous_arm_position = arm_position;

        if (arm_position > ArmFlipPosition && !is_flipped) {
            wrist_target += WristDirection * FlipMotorDegrees;
            is_flipped = true;
        } else if (arm_position < ArmFlipPosition && is_flipped) {
            wrist_target -= WristDirection * FlipMotorDegrees;
            is_flipped = false;
        } else {
            wrist_target -= arm_delta * ArmDirection * WristDirection *
                            WristMotorToArmMotorRatio;
        }

        if (std::abs(arm_delta) > 0.05 || arm_position == ArmFlipPosition) {
            Wrist.move_absolute(wrist_target, WristVelocity);
        }
        pros::delay(UpdatePeriodMs);
    }
}

}

void start() {
    if (running) {
        return;
    }

    running = true;
    if (leveling_task == nullptr) {
        leveling_task = new pros::Task(update, "Claw Leveling Task");
    }
}

void stop() {
    if (!running) {
        return;
    }

    running = false;
}

}