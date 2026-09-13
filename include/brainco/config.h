#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace brainco {

constexpr std::size_t finger_count = 6;
using FingerValues = std::array<double, finger_count>;
enum class InputMode { Auto, Fingers, Gripper };
enum class ControlMode { PositionSpeed, PositionTime };

struct GripperConfig {
    std::size_t index = 0;
    double open_value = 0.0;
    double closed_value = 1.0;
    FingerValues open_pose{0, 0, 0, 0, 0, 0};
    FingerValues closed_pose{1, 1, 1, 1, 1, 1};
    double speed = 0.3;
};

struct HandConfig {
    std::string name;
    uint8_t slave_id;
    std::string port; // Empty: discover among supported serial port names.
    std::string command_topic;
    InputMode input = InputMode::Auto;
    ControlMode control = ControlMode::PositionSpeed;
    uint16_t duration_ms = 100;
    unsigned command_timeout_ms = 0; // Zero preserves indefinite legacy commands.
    bool startup_open = true;
    GripperConfig gripper{};
};

struct DiscoveryConfig {
    uint32_t baudrate = 460800;
    unsigned attempts = 5;
    unsigned probe_attempts = 3;
    unsigned settle_ms = 150;
    unsigned probe_retry_ms = 100;
    unsigned retry_ms = 500;
    bool require_both = false; // Legacy partial startup; dual_revo2.yaml requires both.
};

struct Config {
    DiscoveryConfig discovery;
    unsigned period_ms = 10;
    std::array<HandConfig, 2> hands{{
        {"left", 126, "", "rt/brainco/left/cmd"},
        {"right", 127, "", "rt/brainco/right/cmd"}
    }};
};

// Parsing and validation finish before DDS or serial ports are opened.
Config load_config(const std::string& path);
void validate_config(const Config& config);

} // namespace brainco
