#include "lemlib/pid.hpp"

#include <cmath>

namespace lemlib {
PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

float PID::update(float error) {
    if (signFlipReset && prevError != 0 && (error * prevError < 0)) {
        integral = 0;
    }

    if (windupRange != 0 && std::fabs(error) > windupRange) {
        integral = 0;
    }

    integral += error;
    const float derivative = error - prevError;
    prevError = error;

    return (kP * error) + (kI * integral) + (kD * derivative);
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}
} // namespace lemlib
