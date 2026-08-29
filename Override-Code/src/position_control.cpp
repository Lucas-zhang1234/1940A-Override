#include "position_control.hpp"

#include "pros/rtos.hpp"
#include "robot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace position_control {
namespace {

constexpr std::uint32_t LoopPeriodMs = 10; // how often the loop runs
constexpr double PositionTolerance = 1.0; // the motor is considered "at target" when it is within 1 degree of the pos
constexpr double ClawPositionTolerance = 2.5;
constexpr double ClawMinRelativeAngle = -90.0; // wrist's allowed mechanical range relative to the arm
constexpr double ClawMaxRelativeAngle = 90.0;
constexpr double FlipMarginDegrees = 5.0;
constexpr double Kp = 18.0;
constexpr double Ki = 0.0;
constexpr double Kd = 0.8;
constexpr double IntegralLimit = 500.0;
constexpr double MaxVoltage = 12000.0;
constexpr double ClawKp = 80.0;
constexpr double ClawKd = 1.5;
constexpr double ClawMaxVoltageStep = 1200.0;
constexpr double ArmZeroOffsetDegrees = 1921;
constexpr double ClawZeroOffsetDegrees = -100;
constexpr double DesiredGlobalAngleDegrees = 0.0;

struct Command {
    CommandId id;
    double target;
    std::int32_t max_velocity_rpm;
    std::uint32_t timeout_ms;
    std::uint32_t started_at;
};

struct MotorState {
    pros::Motor* motor;
    double motor_max_rpm;
    std::deque<Command> queue;
    Command active{};
    Status active_status = Status::Pending;
    double integral = 0.0;
    double previous_error = 0.0;
    bool has_active = false;
};

struct CommandResult {
    CommandId id = 0;
    Status status = Status::Invalid;
};

std::array<MotorState, 3> states{{
    {&Lift, 200.0},
    {&Arm, 600.0},
    {&Wrist, 600.0}
}};
std::array<CommandResult, 32> command_results{};
std::size_t command_result_count = 0;
pros::Mutex state_mutex;
pros::Task* control_task = nullptr;
CommandId next_command_id = 1;

double desired_global_angle = DesiredGlobalAngleDegrees;
double arm_zero_offset = ArmZeroOffsetDegrees;
double claw_zero_offset = ClawZeroOffsetDegrees;
ClawMode claw_mode = ClawMode::NORMAL;
ClawMode flip_destination = ClawMode::FLIPPED;
double claw_integral = 0.0;
double claw_previous_error = 0.0;
double claw_previous_output = 0.0;
bool claw_was_enabled = false;

std::size_t index_for(MotorId motor) {
    return static_cast<std::size_t>(motor);
}

bool valid_motor(MotorId motor) {
    return index_for(motor) < states.size();
}

double normalize_angle(double angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }
    while (angle <= -180.0) {
        angle += 360.0;
    }
    return angle;
}

bool reachable(double target) {
    return target >= ClawMinRelativeAngle && target <= ClawMaxRelativeAngle;
}

bool safely_reachable(double target) {
    return target >= ClawMinRelativeAngle + FlipMarginDegrees &&
           target <= ClawMaxRelativeAngle - FlipMarginDegrees;
}

ClawMode opposite(ClawMode mode) {
    return mode == ClawMode::NORMAL ? ClawMode::FLIPPED : ClawMode::NORMAL;
}

double target_for_mode(double arm_angle, ClawMode mode) {
    const double mode_offset = mode == ClawMode::FLIPPED ? 180.0 : 0.0;
    return normalize_angle(desired_global_angle + mode_offset - arm_angle);
}

void save_result(CommandId id, Status status) {
    if (command_result_count < command_results.size()) {
        command_results[command_result_count++] = {id, status};
    } else {
        command_results[id % command_results.size()] = {id, status};
    }
}

void finish(MotorState& state, Status status) {
    state.motor->brake();
    state.active_status = status;
    save_result(state.active.id, status);
    state.has_active = false;
    state.integral = 0.0;
    state.previous_error = 0.0;
}

std::int32_t pid_output(double error, double& integral, double& previous_error,
                        double max_velocity_rpm, double motor_max_rpm) {
    integral = std::clamp(integral + error * (LoopPeriodMs / 1000.0),
                          -IntegralLimit, IntegralLimit);
    const double derivative = (error - previous_error) / (LoopPeriodMs / 1000.0);
    previous_error = error;
    const double velocity_limit = std::clamp(max_velocity_rpm / motor_max_rpm, 0.05, 1.0);
    const double output = std::clamp((Kp * error) + (Ki * integral) + (Kd * derivative),
                                     -MaxVoltage * velocity_limit,
                                     MaxVoltage * velocity_limit);
    return static_cast<std::int32_t>(output);
}

void clawPIDUnlocked(double target) {
    // Never command outside the physical relative range, even during a flip.
    target = std::clamp(target, ClawMinRelativeAngle, ClawMaxRelativeAngle);
    const double claw_angle = Wrist.get_position() - claw_zero_offset;
    const double error = target - claw_angle;
    if (std::abs(error) <= ClawPositionTolerance) {
        Wrist.brake();
        claw_integral = 0.0;
        claw_previous_error = error;
        claw_previous_output = 0.0;
        return;
    }
    claw_integral = std::clamp(claw_integral + error * (LoopPeriodMs / 1000.0),
                               -IntegralLimit, IntegralLimit);
    const double derivative = (error - claw_previous_error) / (LoopPeriodMs / 1000.0);
    claw_previous_error = error;
    const double requested_output = std::clamp(
        (ClawKp * error) + (ClawKd * derivative), -MaxVoltage, MaxVoltage);
    const double output = std::clamp(requested_output,
                                     claw_previous_output - ClawMaxVoltageStep,
                                     claw_previous_output + ClawMaxVoltageStep);
    claw_previous_output = output;
    Wrist.move_voltage(static_cast<std::int32_t>(output));
}

void updateClawCompensationUnlocked() {
    // The current project maps extended Fingers to the closed clamp state.
    const bool enabled = Fingers.is_extended();
    if (!enabled) {
        Wrist.brake();
        claw_integral = 0.0;
        claw_previous_error = 0.0;
        claw_previous_output = 0.0;
        claw_was_enabled = false;
        return;
    }

    const double arm_angle = getArmAngle();
    if (!claw_was_enabled) {
        const double normal_target = target_for_mode(arm_angle, ClawMode::NORMAL);
        claw_mode = reachable(normal_target) ? ClawMode::NORMAL : ClawMode::FLIPPED;
        claw_was_enabled = true;
        claw_integral = 0.0;
        claw_previous_error = 0.0;
        claw_previous_output = 0.0;
    }

    if (claw_mode == ClawMode::NORMAL || claw_mode == ClawMode::FLIPPED) {
        const double current_target = target_for_mode(arm_angle, claw_mode);
        const ClawMode other_mode = opposite(claw_mode);
        const double other_target = target_for_mode(arm_angle, other_mode);
        if (!reachable(current_target) && safely_reachable(other_target)) {
            // The destination has a 5-degree safety margin. On the way back,
            // the opposite mode must also be 5 degrees inside its limit. This
            // creates a 10-degree arm hysteresis band around the transition.
            flip_destination = other_mode;
            claw_mode = ClawMode::FLIPPING;
            claw_integral = 0.0;
            claw_previous_error = 0.0;
        } else {
            clawPIDUnlocked(current_target);
            return;
        }
    }

    const double flip_target = target_for_mode(arm_angle, flip_destination);
    if (reachable(flip_target)) {
        clawPIDUnlocked(flip_target);
        const double error = flip_target - (Wrist.get_position() - claw_zero_offset);
        if (std::abs(error) <= ClawPositionTolerance) {
            claw_mode = flip_destination;
            claw_integral = 0.0;
            claw_previous_error = error;
        }
    } else {
        // If the arm moves during the flip, remain inside the physical range.
        clawPIDUnlocked(std::clamp(flip_target, ClawMinRelativeAngle, ClawMaxRelativeAngle));
    }
}

void control_loop() {
    while (true) {
        const std::uint32_t now = pros::millis();
        state_mutex.take();
        const bool wrist_command_active = !Fingers.is_extended();

        for (MotorState& state : states) {
            if (state.motor == &Wrist && !wrist_command_active) {
                continue;
            }
            if (!state.has_active) {
                if (state.queue.empty()) {
                    continue;
                }
                state.active = state.queue.front();
                state.queue.pop_front();
                state.active.started_at = now;
                state.active_status = Status::Running;
                state.has_active = true;
                state.integral = 0.0;
                state.previous_error = 0.0;
            }

            const double error = state.active.target - state.motor->get_position();
            if (std::abs(error) <= PositionTolerance) {
                finish(state, Status::Completed);
                continue;
            }
            if (now - state.active.started_at >= state.active.timeout_ms) {
                finish(state, Status::TimedOut);
                continue;
            }
            state.motor->move_voltage(pid_output(
                error, state.integral, state.previous_error,
                static_cast<double>(state.active.max_velocity_rpm), state.motor_max_rpm));
        }

        if (!wrist_command_active) {
            updateClawCompensationUnlocked();
        }
        state_mutex.give();
        pros::delay(LoopPeriodMs);
    }
}

CommandId enqueue(MotorId motor, double target, std::int32_t max_velocity_rpm,
                  std::uint32_t timeout_ms) {
    if (!valid_motor(motor) || !std::isfinite(target) || max_velocity_rpm <= 0 || timeout_ms == 0) {
        return 0;
    }
    start();
    state_mutex.take();
    const CommandId id = next_command_id++;
    states[index_for(motor)].queue.push_back({id, target, max_velocity_rpm, timeout_ms, 0});
    state_mutex.give();
    return id;
}

Status wait_for(CommandId command, std::uint32_t timeout_ms) {
    const std::uint32_t started_at = pros::millis();
    while (pros::millis() - started_at < timeout_ms) {
        const Status status = get_status(command);
        if (status == Status::Completed || status == Status::TimedOut ||
            status == Status::Cancelled || status == Status::Invalid) {
            return status;
        }
        pros::delay(LoopPeriodMs);
    }
    cancel(command);
    return Status::TimedOut;
}

}

double getArmAngle() {
    return Arm.get_position() - arm_zero_offset;
}

double calculateClawTarget(double arm_angle, double desired_global_angle) {
    return normalize_angle(desired_global_angle - arm_angle);
}

void clawPID(double target) {
    state_mutex.take();
    clawPIDUnlocked(target);
    state_mutex.give();
}

void updateClawCompensation() {
    state_mutex.take();
    updateClawCompensationUnlocked();
    state_mutex.give();
}

void setDesiredGlobalAngle(double angle) {
    state_mutex.take();
    desired_global_angle = normalize_angle(angle);
    state_mutex.give();
}

void setArmZeroOffset(double offset) {
    state_mutex.take();
    arm_zero_offset = offset;
    state_mutex.give();
}

void setClawZeroOffset(double offset) {
    state_mutex.take();
    claw_zero_offset = offset;
    state_mutex.give();
}

ClawMode getClawMode() {
    state_mutex.take();
    const ClawMode mode = claw_mode;
    state_mutex.give();
    return mode;
}

void start() {
    if (control_task == nullptr) {
        control_task = new pros::Task(control_loop, "Position Control Task");
    }
}

CommandId move_absolute(MotorId motor, double position, std::int32_t max_velocity_rpm,
                        std::uint32_t timeout_ms) {
    return enqueue(motor, position, max_velocity_rpm, timeout_ms);
}

CommandId move_relative(MotorId motor, double delta, std::int32_t max_velocity_rpm,
                        std::uint32_t timeout_ms) {
    if (!valid_motor(motor) || !std::isfinite(delta)) {
        return 0;
    }
    state_mutex.take();
    const double target = states[index_for(motor)].motor->get_position() + delta;
    state_mutex.give();
    return enqueue(motor, target, max_velocity_rpm, timeout_ms);
}

CommandId move_absolute_degrees(MotorId motor, double position_degrees,
                                std::int32_t max_velocity_rpm, std::uint32_t timeout_ms) {
    return move_absolute(motor, position_degrees, max_velocity_rpm, timeout_ms);
}

CommandId move_relative_degrees(MotorId motor, double delta_degrees,
                                std::int32_t max_velocity_rpm, std::uint32_t timeout_ms) {
    return move_relative(motor, delta_degrees, max_velocity_rpm, timeout_ms);
}

Status get_status(CommandId command) {
    if (command == 0) {
        return Status::Invalid;
    }
    state_mutex.take();
    for (const MotorState& state : states) {
        if (state.has_active && state.active.id == command) {
            const Status status = state.active_status;
            state_mutex.give();
            return status;
        }
        for (const Command& queued : state.queue) {
            if (queued.id == command) {
                state_mutex.give();
                return Status::Pending;
            }
        }
    }
    for (const CommandResult& result : command_results) {
        if (result.id == command) {
            state_mutex.give();
            return result.status;
        }
    }
    state_mutex.give();
    return Status::Invalid;
}

bool cancel(CommandId command) {
    if (command == 0) {
        return false;
    }
    state_mutex.take();
    for (MotorState& state : states) {
        if (state.has_active && state.active.id == command) {
            finish(state, Status::Cancelled);
            state_mutex.give();
            return true;
        }
        for (auto queued = state.queue.begin(); queued != state.queue.end(); ++queued) {
            if (queued->id == command) {
                state.queue.erase(queued);
                save_result(command, Status::Cancelled);
                state_mutex.give();
                return true;
            }
        }
    }
    state_mutex.give();
    return false;
}

Status move_absolute_blocking(MotorId motor, double position, std::int32_t max_velocity_rpm,
                              std::uint32_t timeout_ms) {
    return wait_for(move_absolute(motor, position, max_velocity_rpm, timeout_ms), timeout_ms);
}

Status move_relative_blocking(MotorId motor, double delta, std::int32_t max_velocity_rpm,
                              std::uint32_t timeout_ms) {
    return wait_for(move_relative(motor, delta, max_velocity_rpm, timeout_ms), timeout_ms);
}

Status move_absolute_degrees_blocking(MotorId motor, double position_degrees,
                                      std::int32_t max_velocity_rpm, std::uint32_t timeout_ms) {
    return wait_for(move_absolute_degrees(motor, position_degrees, max_velocity_rpm, timeout_ms),
                    timeout_ms);
}

Status move_relative_degrees_blocking(MotorId motor, double delta_degrees,
                                      std::int32_t max_velocity_rpm, std::uint32_t timeout_ms) {
    return wait_for(move_relative_degrees(motor, delta_degrees, max_velocity_rpm, timeout_ms),
                    timeout_ms);
}

}
