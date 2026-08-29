#include "position_control.hpp"

#include "pros/rtos.hpp"
#include "robot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

namespace position_control {
namespace {

constexpr std::uint32_t LoopPeriodMs = 20;
constexpr double PositionTolerance = 2.0;
constexpr double MaxVoltage = 12000.0;

struct PidGains {
    double kp;
    double ki;
    double kd;
    double integral_limit;
};

constexpr PidGains ArmAndLiftGains{160.0, 25.5, 10.0, 500.0};
constexpr PidGains WristGains{160.0, 0.0, 20.0, 0.0};

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
    PidGains gains;
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
    {&Lift, 200.0, ArmAndLiftGains},
    {&Arm, 600.0, ArmAndLiftGains},
    {&Wrist, 600.0, WristGains}
}};

std::array<CommandResult, 32> command_results{};
std::size_t command_result_count = 0;
pros::Mutex state_mutex;
pros::Task* control_task = nullptr;
CommandId next_command_id = 1;

std::size_t index_for(MotorId motor) {
    return static_cast<std::size_t>(motor);
}

bool valid_motor(MotorId motor) {
    return index_for(motor) < states.size();
}

void finish(MotorState& state, Status status) {
    state.motor->brake();
    state.active_status = status;
    if (command_result_count < command_results.size()) {
        command_results[command_result_count++] = {state.active.id, status};
    } else {
        command_results[state.active.id % command_results.size()] = {state.active.id, status};
    }
    state.has_active = false;
    state.integral = 0.0;
    state.previous_error = 0.0;
}

void control_loop() {
    while (true) {
        const std::uint32_t now = pros::millis();
        state_mutex.take();

        for (MotorState& state : states) {
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

            state.integral = std::clamp(
                state.integral + error * (LoopPeriodMs / 1000.0),
                -state.gains.integral_limit, state.gains.integral_limit);
            const double derivative = (error - state.previous_error) / (LoopPeriodMs / 1000.0);
            state.previous_error = error;

            const double velocity_limit = std::clamp(
                static_cast<double>(state.active.max_velocity_rpm) / state.motor_max_rpm, 0.05, 1.0);
            const double output = std::clamp(
                (state.gains.kp * error) +
                    (state.gains.ki * state.integral) +
                    (state.gains.kd * derivative),
                -MaxVoltage * velocity_limit, MaxVoltage * velocity_limit);
            state.motor->move_voltage(static_cast<std::int32_t>(output));
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

void start() {
    if (control_task == nullptr) {
        control_task = new pros::Task(control_loop, "Position Control Task");
    }
}

void clear_target(MotorId motor) {
    if (!valid_motor(motor)) {
        return;
    }

    state_mutex.take();
    MotorState& state = states[index_for(motor)];
    state.queue.clear();
    state.has_active = false;
    state.active_status = Status::Cancelled;
    state.integral = 0.0;
    state.previous_error = 0.0;
    state.motor->brake();
    state_mutex.give();
}

void set_target(MotorId motor, double position, std::int32_t max_velocity_rpm,
                std::uint32_t timeout_ms) {
    if (!valid_motor(motor) || !std::isfinite(position) || max_velocity_rpm <= 0 || timeout_ms == 0) {
        return;
    }

    start();
    state_mutex.take();
    MotorState& state = states[index_for(motor)];
    state.queue.clear();
    if (!state.has_active || state.active.id != 0) {
        state.active = {0, position, max_velocity_rpm, timeout_ms, pros::millis()};
        state.active_status = Status::Running;
        state.has_active = true;
        state.integral = 0.0;
        state.previous_error = 0.0;
    } else {
        state.active.target = position;
        state.active.max_velocity_rpm = max_velocity_rpm;
        state.active.timeout_ms = timeout_ms;
        state.active.started_at = pros::millis();
    }
    state_mutex.give();
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
    return enqueue(motor, target,
                   max_velocity_rpm, timeout_ms);
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
        if (!state.has_active && state.active.id == command) {
            const Status status = state.active_status;
            state_mutex.give();
            return status;
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
                state.active = {command, 0, 0, 0, 0};
                state.active_status = Status::Cancelled;
                if (command_result_count < command_results.size()) {
                    command_results[command_result_count++] = {command, Status::Cancelled};
                } else {
                    command_results[command % command_results.size()] = {command, Status::Cancelled};
                }
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