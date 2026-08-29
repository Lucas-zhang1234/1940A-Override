#ifndef CLAW_LEVELING_HPP
#define CLAW_LEVELING_HPP

#include "pros/motors.hpp"

#include <cmath>

namespace claw_leveling {

class ArmWristController {
public:
    ArmWristController(pros::Motor& armMotor,
                       pros::Motor& wristMotor,
                       double wristMinDeg = -90.0,
                       double wristMaxDeg = 90.0,
                       bool armReversed = false,
                       bool wristReversed = false,
                       int wristVelocity = 100,
                       double wristZeroOffsetDeg = 0.0);

    void update();
    void zero();

    void setArmVoltageMv(int mv);
    void setArmVelocity(int pct);
    void setArmTargetAngle(double deg, int velocity = 100);
    void setWristVelocityLimit(int pct);

private:
    pros::Motor& armMotor_;
    pros::Motor& wristMotor_;

    double wristMin_;
    double wristMax_;
    int wristVelocity_ = 100;
    double wristZeroOffsetDeg_ = 0.0;

    static double normalizeSigned90Deg(double deg);
    double convertToMotorDeg(double wristPhysicalDeg) const;
    double computeWristTarget(double armAngleDeg);
};

void start();
void stop();

}

#endif