#pragma once

#include "brainco/config.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <vector>

namespace brainco {

struct MotorInput { double q = 0; double dq = 0; };
struct HandCommand {
    std::array<uint16_t, finger_count> positions{};
    std::array<uint16_t, finger_count> speeds{};
};

inline uint16_t normalized(double value) {
    return static_cast<uint16_t>(std::clamp(value, 0.0, 1.0) * 1000.0);
}

inline uint16_t legacy_normalized(double value) {
    // Preserve the original float arithmetic for MotorCmds q/dq (e.g. 0.7f -> 700).
    return static_cast<uint16_t>(std::clamp(static_cast<float>(value), 0.f, 1.f) * 1000.f);
}

// DDS-independent retargeting boundary: future message adapters supply MotorInput.
inline std::optional<HandCommand> retarget(const HandConfig& config,
                                          const std::vector<MotorInput>& input) {
    auto mode = config.input;
    if (mode == InputMode::Auto) {
        // Message length is the wire contract. Never infer a gripper from q values.
        if (input.size() == finger_count) mode = InputMode::Fingers;
        else if (input.size() == 1) mode = InputMode::Gripper;
        else return std::nullopt;
    }
    HandCommand result;
    if (mode == InputMode::Fingers) {
        if (input.size() != finger_count) return std::nullopt;
        for (std::size_t i = 0; i < finger_count; ++i) {
            if (!std::isfinite(input[i].q) ||
                (config.control == ControlMode::PositionSpeed && !std::isfinite(input[i].dq)))
                return std::nullopt;
            result.positions[i] = legacy_normalized(input[i].q);
            result.speeds[i] = config.control == ControlMode::PositionSpeed
                ? legacy_normalized(input[i].dq) : 0;
        }
    } else {
        const auto index = config.input == InputMode::Auto ? 0 : config.gripper.index;
        if (index >= input.size()) return std::nullopt;
        const double value = input[index].q;
        if (!std::isfinite(value)) return std::nullopt;
        const auto& g = config.gripper;
        const double closure = std::clamp((value - g.open_value) /
                                         (g.closed_value - g.open_value), 0.0, 1.0);
        for (std::size_t i = 0; i < finger_count; ++i) {
            result.positions[i] = normalized(g.open_pose[i] + closure * (g.closed_pose[i] - g.open_pose[i]));
            result.speeds[i] = normalized(g.speed);
        }
    }
    return result;
}

using Clock = std::chrono::steady_clock;
struct CommandSample {
    HandCommand command;
    Clock::time_point received;
    uint64_t sequence;
};

class CommandBuffer {
public:
    explicit CommandBuffer(HandConfig config) : config_(std::move(config)) {
        if (config_.startup_open) {
            HandCommand command;
            command.speeds.fill(1000);
            latest_ = CommandSample{command, Clock::now(), ++sequence_};
        }
    }

    bool accept(const std::vector<MotorInput>& input, Clock::time_point now = Clock::now()) {
        const auto command = retarget(config_, input);
        if (!command) return false; // Invalid messages must not refresh the watchdog.
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = CommandSample{*command, now, ++sequence_};
        return true;
    }

    std::optional<CommandSample> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }

private:
    const HandConfig config_;
    mutable std::mutex mutex_;
    uint64_t sequence_ = 0;
    std::optional<CommandSample> latest_;
};

enum class CommandAction { None, Send, Stop };
class CommandSchedule {
public:
    CommandAction next(const HandConfig& config, const std::optional<CommandSample>& sample,
                       Clock::time_point now = Clock::now()) {
        if (!sample) return CommandAction::None;
        if (config.command_timeout_ms > 0 &&
            now - sample->received >= std::chrono::milliseconds(config.command_timeout_ms)) {
            if (!sent_ || stopped_) return CommandAction::None;
            stopped_ = true;
            return CommandAction::Stop;
        }
        // Time-based trajectories are submitted once per accepted DDS sample.
        if (config.control == ControlMode::PositionTime && sent_ &&
            sample->sequence == last_sequence_) return CommandAction::None;
        last_sequence_ = sample->sequence;
        sent_ = true;
        stopped_ = false;
        return CommandAction::Send;
    }
private:
    bool sent_ = false;
    bool stopped_ = false;
    uint64_t last_sequence_ = 0;
};

} // namespace brainco
