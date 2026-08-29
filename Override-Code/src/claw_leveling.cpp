#include "claw_leveling.hpp"

#include "position_control.hpp"
#include "pros/rtos.hpp"
#include "robot.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace claw_leveling {
namespace {

constexpr double ArmZero = 0.0;
constexpr double WristZero = 0.0;
constexpr double ArmDirection = 1.0;
constexpr double WristDirection = 1.0;
constexpr double LevelAngle = 0.0;
constexpr double WristMinimum = -100.0;
constexpr double WristMaximum = 100.0;
constexpr std::int32_t WristVelocity = 300;
constexpr std::uint32_t WristTimeoutMs = 250;
constexpr std::uint32_t UpdatePeriodMs = 40;
constexpr double TargetDeadband = 1.0;

pros::Task* leveling_task = nullptr;
bool running = false;

double nearest_equivalent(double target, double current) {
    return target + 360.0 * std::round((current - target) / 360.0);
}

double wrist_target_for(double arm_position, double desired_claw_angle) {
    const double arm_angle = ArmDirection * (arm_position - ArmZero);
    const double wrist_angle = desired_claw_angle - arm_angle;
    return WristZero + WristDirection * wrist_angle;
}

bool in_wrist_range(double target) {
    return target >= WristMinimum && target <= WristMaximum;
}

double select_target(double arm_position, double current_wrist) {
    double selected = std::numeric_limits<double>::quiet_NaN();
    double distance = std::numeric_limits<double>::infinity();

    for (double orientation : std::array<double, 2>{LevelAngle, LevelAngle + 180.0}) {
        const double raw_target = wrist_target_for(arm_position, orientation);
        const double target = nearest_equivalent(raw_target, current_wrist);
        if (in_wrist_range(target) && std::abs(target - current_wrist) < distance) {
            selected = target;
            distance = std::abs(target - current_wrist);
        }
    }
    return selected;
}

void update() {
    double previous_target = std::numeric_limits<double>::quiet_NaN();
    while (true) {
        if (!running) {
            previous_target = std::numeric_limits<double>::quiet_NaN();
            pros::delay(UpdatePeriodMs);
            continue;
        }

        const double arm_position = Arm.get_position();
        const double wrist_position = Wrist.get_position();
        const double target = select_target(arm_position, wrist_position);

        if (std::isfinite(target) &&
            (!std::isfinite(previous_target) || std::abs(target - previous_target) >= TargetDeadband)) {
            position_control::set_target(position_control::MotorId::Wrist, target,
                                          WristVelocity, WristTimeoutMs);
            previous_target = target;
        } else if (std::isfinite(target)) {
            position_control::set_target(position_control::MotorId::Wrist, previous_target,
                                          WristVelocity, WristTimeoutMs);
        }
        pros::delay(UpdatePeriodMs);
    }
}

}

void start() {
    if (leveling_task == nullptr) {
        running = true;
        leveling_task = new pros::Task(update, "Claw Leveling Task");
    }
}

void stop() {
    running = false;
}

}