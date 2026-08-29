#include "claw_leveling.hpp"

#include "pros/rtos.hpp"
#include "robot.hpp"

#include <algorithm>
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
                                       int wristVelocity,
                                       double wristZeroOffsetDeg)
    : armMotor_(armMotor),
      wristMotor_(wristMotor),
      wristMin_(wristMinDeg),
      wristMax_(wristMaxDeg),
      wristVelocity_(wristVelocity),
      wristZeroOffsetDeg_(wristZeroOffsetDeg) {
    armMotor_.set_reversed(armReversed);
    wristMotor_.set_reversed(wristReversed);
    armMotor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wristMotor_.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void ArmWristController::update() {
    const double armPhysicalDeg = armMotor_.get_position();
    const double targetPhysicalDeg = computeWristTarget(armPhysicalDeg);
    const double targetMotorDeg = convertToMotorDeg(targetPhysicalDeg);

    pros::screen::print(pros::E_TEXT_MEDIUM, 4,
                       "Arm phys deg: %.2f deg", armPhysicalDeg);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,
                       "Wrist rel: %.2f deg", targetPhysicalDeg);
    pros::screen::print(pros::E_TEXT_MEDIUM, 6,
                       "Wrist motor: %.2f deg", targetMotorDeg);

    wristMotor_.move_absolute(targetMotorDeg, wristVelocity_);
}

void ArmWristController::zero() {
    armMotor_.set_zero_position(0);
    wristMotor_.set_zero_position(0);
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

double ArmWristController::normalizeSigned90Deg(double deg) {
    while (deg > 90.0) {
        deg -= 180.0;
    }
    while (deg < -90.0) {
        deg += 180.0;
    }
    return deg;
}

double ArmWristController::convertToMotorDeg(double wristPhysicalDeg) const {
    return wristPhysicalDeg + wristZeroOffsetDeg_;
}

double ArmWristController::computeWristTarget(double armAngleDeg) {
    double targetPhysicalDeg = normalizeSigned90Deg(-armAngleDeg);

    if (targetPhysicalDeg > wristMax_) {
        return wristMax_;
    }
    if (targetPhysicalDeg < wristMin_) {
        return wristMin_;
    }
    return targetPhysicalDeg;
}

void start() {
    if (g_controller == nullptr) {
        g_controller = new ArmWristController(Arm, Wrist, -90.0, 90.0, false, false, 100, 0.0);
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