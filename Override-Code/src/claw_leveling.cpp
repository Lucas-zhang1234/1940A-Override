#include "claw_leveling.hpp"

#include "position_control.hpp"
#include "pros/rtos.hpp"
#include "robot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace claw_leveling {
namespace {

constexpr double ArmDirection = 1.0;
constexpr double WristDirection = 1.0;
constexpr double ArmMotorToJointRatio = 5.0;
constexpr double WristMotorToJointRatio = 3.5;
constexpr double WristMotorToArmMotorRatio =
    WristMotorToJointRatio / ArmMotorToJointRatio;
constexpr double WristMinimum = -250.0;
constexpr double WristMaximum = 250.0;
constexpr double MinimumArmDelta = 0.05;
constexpr std::int32_t WristVelocity = 12000;
constexpr std::uint32_t UpdatePeriodMs = 20;

pros::Task* leveling_task = nullptr;
bool running = false;

double clamp_wrist_delta(double delta, double wrist_position) {
    return std::clamp(delta,
                      WristMinimum - wrist_position,
                      WristMaximum - wrist_position);
}

void update() {
    bool has_previous_arm_position = false;
    double previous_arm_position = 0.0;
    while (true) {
        if (!running) {
            has_previous_arm_position = false;
            pros::delay(UpdatePeriodMs);
            continue;
        }

        const double arm_position = Arm.get_position();
        if (!has_previous_arm_position) {
            previous_arm_position = arm_position;
            has_previous_arm_position = true;
            pros::delay(UpdatePeriodMs);
            continue;
        }

        const double arm_delta = arm_position - previous_arm_position;
        previous_arm_position = arm_position;

        if (std::abs(arm_delta) >= MinimumArmDelta) {
            const double wrist_delta = clamp_wrist_delta(
                -arm_delta * ArmDirection * WristDirection *
                    WristMotorToArmMotorRatio,
                Wrist.get_position());

            if (std::abs(wrist_delta) >= MinimumArmDelta) {
                Wrist.move_relative(wrist_delta, WristVelocity);
            }
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
    position_control::clear_target(position_control::MotorId::Wrist);
    if (leveling_task == nullptr) {
        leveling_task = new pros::Task(update, "Claw Leveling Task");
    }
}

void stop() {
    if (!running) {
        return;
    }

    running = false;
    position_control::clear_target(position_control::MotorId::Wrist);
}

}