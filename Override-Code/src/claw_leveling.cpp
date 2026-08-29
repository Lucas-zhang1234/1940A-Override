#include "claw_leveling.hpp"

#include "position_control.hpp"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "pros/screen.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace claw_leveling {
namespace {

constexpr double ArmZero = 204.4; // in motor encoder degrees
constexpr double WristZero = 222.8; // in motor encoder degrees
constexpr double ArmDirection = 1.0;
constexpr double WristDirection = 1.0;
constexpr double LevelAngle = 0.0;
constexpr double WristMinimum = -350.0; // in motor encoder degrees
constexpr double WristMaximum = 280.0; // in motor encoder degrees
constexpr double ArmMotorToJointRatio = 5.0; // in motor encoder degrees per joint degree
constexpr double WristMotorToJointRatio = 3.5; // in motor encoder degrees per joint degree
constexpr std::int32_t WristVelocity = 600;
constexpr std::uint32_t WristTimeoutMs = 250;
constexpr std::uint32_t UpdatePeriodMs = 40;
constexpr double TargetDeadband = 1.0;

pros::Task* leveling_task = nullptr;
bool running = false;

double nearest_equivalent(double target, double current) {
    const double motorFullRotation = 360.0 * WristMotorToJointRatio;
    return target + motorFullRotation * std::round((current - target) / motorFullRotation);
}

double wrist_target_for(double arm_position, double desired_claw_angle) {
    const double arm_angle = ArmDirection * (arm_position - ArmZero) / ArmMotorToJointRatio;
    const double required_wrist_angle = desired_claw_angle - arm_angle;
    return WristZero + WristDirection * required_wrist_angle * WristMotorToJointRatio;
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

        pros::screen::print(
            pros::E_TEXT_MEDIUM,
			3,
			"A %.1f W %.1f T %.1f",
			arm_position,
			wrist_position,
			target
		);

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
    running = true;
    if (leveling_task == nullptr) {
        leveling_task = new pros::Task(update, "Claw Leveling Task");
    }
}

void stop() {
    running = false;
}

}