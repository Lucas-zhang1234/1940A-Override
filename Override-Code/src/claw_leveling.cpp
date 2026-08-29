#include "claw_leveling.hpp"

#include "pros/rtos.hpp"
#include "robot.hpp"

#include <cmath>
#include <cstdint>

namespace claw_leveling {
namespace {

constexpr std::uint32_t kUpdatePeriodMs = 20;

pros::Task* g_levelingTask = nullptr;
bool g_running = false;
ArmWristController* g_controller = nullptr;

void updateTask() {
    while (g_running) {
        if (g_controller != nullptr) {
            g_controller->update();
        }
        pros::delay(kUpdatePeriodMs);
    }
}

}

ArmWristController::ArmWristController(pros::Motor& armMotor,
                                       pros::Motor& wristMotor,
                                       double wristMinDeg,
                                       double wristMaxDeg,
                                       bool armReversed,
                                       bool wristReversed,
                                       int wristVelocity)
    : armMotor_(armMotor),
      wristMotor_(wristMotor),
      wristMin_(wristMinDeg),
      wristMax_(wristMaxDeg),
      wristVelocity_(wristVelocity) {
    armMotor_.set_reversed(armReversed);
    wristMotor_.set_reversed(wristReversed);
    armMotor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wristMotor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void ArmWristController::update() {
    const double armPhysicalDeg = armMotor_.get_position() / 5.0;
    const double targetPhysicalDeg = computeWristTarget(armPhysicalDeg);
    const double targetMotorDeg = targetPhysicalDeg * 2.0;

    pros::screen::print(pros::E_TEXT_MEDIUM, 4,
                       "Wrist abs rel: %.2f deg", std::abs(targetPhysicalDeg));
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,
                       "Wrist motor: %.2f deg", targetMotorDeg);

    wristMotor_.move_absolute(targetMotorDeg, wristVelocity_);
}

void ArmWristController::zero() {
    armMotor_.set_zero_position(0);
    wristMotor_.set_zero_position(0);
    wristFlipped_ = false;
}

void ArmWristController::setArmVoltageMv(int mv) {
    armMotor_.move_voltage(mv);
}

void ArmWristController::setArmVelocity(int pct) {
    armMotor_.move_velocity(pct);
}

void ArmWristController::setArmTargetAngle(double deg, int velocity) {
    armMotor_.move_absolute(deg, velocity);
}

void ArmWristController::setWristVelocityLimit(int pct) {
    wristVelocity_ = pct;
}

double ArmWristController::wrap180(double deg) {
    double a = std::fmod(deg, 180.0);
    if (a < 0.0) {
        a += 180.0;
    }
    return a;
}

double ArmWristController::computeWristTarget(double armAngleDeg) {
    double base = wrap180(-armAngleDeg);
    double alt = base + 180.0;

    double baseMotor = base * 2.0;
    double altMotor = alt * 2.0;

    if (baseMotor > 180.0) {
        baseMotor -= 360.0;
    }
    if (altMotor > 180.0) {
        altMotor -= 360.0;
    }

    const bool baseValid =
        (baseMotor >= wristMin_ + (wristFlipped_ ? kHysteresis : 0.0)) &&
        (baseMotor <= wristMax_);
    const bool altValid =
        (altMotor >= wristMin_) &&
        (altMotor <= wristMax_ - (!wristFlipped_ ? kHysteresis : 0.0));

    if (!wristFlipped_) {
        if (!baseValid && altValid) {
            wristFlipped_ = true;
        }
    } else {
        if (!altValid && baseValid) {
            wristFlipped_ = false;
        }
    }

    return wristFlipped_ ? altMotor / 2.0 : baseMotor / 2.0;
}

void start() {
    if (g_controller == nullptr) {
        g_controller = new ArmWristController(Arm, Wrist, -180.0, 180.0, false, false, 100);
    }

    if (g_running) {
        return;
    }

    g_running = true;
    g_levelingTask = new pros::Task(updateTask, "Arm Wrist Leveling Task");
}

void stop() {
    g_running = false;
    g_levelingTask = nullptr;
}

}