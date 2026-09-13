#include "brainco/config.h"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace brainco {
namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::invalid_argument(message);
}

void keys(const YAML::Node& node, std::initializer_list<std::string> allowed) {
    if (!node) return;
    require(node.IsMap(), "Configuration sections must be maps");
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        bool found = false;
        for (const auto& candidate : allowed) found |= key == candidate;
        require(found, "Unknown configuration key: " + key);
    }
}

template<typename T>
void read(const YAML::Node& node, const char* key, T& value) {
    if (node[key]) value = node[key].as<T>();
}

void read_unsigned(const YAML::Node& node, const char* key, unsigned& value) {
    if (!node[key]) return;
    const auto number = node[key].as<long long>();
    require(number >= 0 && number <= std::numeric_limits<unsigned>::max(),
            std::string(key) + " must be a nonnegative integer");
    value = static_cast<unsigned>(number);
}

void pose(const YAML::Node& node, const char* key, FingerValues& value) {
    if (!node[key]) return;
    require(node[key].IsSequence() && node[key].size() == finger_count,
            std::string(key) + " must contain exactly six normalized positions");
    for (std::size_t i = 0; i < finger_count; ++i) value[i] = node[key][i].as<double>();
}

} // namespace

void validate_config(const Config& config) {
    const auto& d = config.discovery;
    require(d.baudrate > 0, "baudrate must be positive");
    require(d.attempts > 0 && d.attempts <= 100, "attempts must be in [1, 100]");
    require(d.probe_attempts > 0 && d.probe_attempts <= 20, "probe_attempts must be in [1, 20]");
    require(d.settle_ms <= 60000 && d.retry_ms <= 60000 && d.probe_retry_ms <= 60000,
            "discovery delays must be at most 60000 ms");
    require(config.period_ms > 0 && config.period_ms <= 1000, "period_ms must be in [1, 1000]");
    require(config.hands[0].slave_id != config.hands[1].slave_id, "Left/right slave IDs must differ");
    for (const auto& h : config.hands) {
        require(h.slave_id > 0 && h.slave_id < 255, "slave_id must be in [1, 254]");
        require(!h.command_topic.empty(), "command_topic cannot be empty");
        require(h.duration_ms > 0 && h.duration_ms <= 2000, "duration_ms must be in [1, 2000]");
        const auto& g = h.gripper;
        require(h.input != InputMode::Auto || g.index == 0,
                "auto input requires gripper.index: 0; use input: gripper for other indices");
        require(std::isfinite(g.open_value) && std::isfinite(g.closed_value) &&
                std::isfinite(g.closed_value - g.open_value) && g.open_value != g.closed_value,
                "Gripper open_value and closed_value must be distinct finite values");
        require(std::isfinite(g.speed) && g.speed >= 0.001 && g.speed <= 1.0,
                "Gripper speed must be in [0.001, 1]; zero means maximum speed on Revo2");
        for (const auto& p : {g.open_pose, g.closed_pose})
            for (double v : p) require(std::isfinite(v) && v >= 0 && v <= 1,
                                     "Gripper pose values must be finite and in [0, 1]");
        require(h.input != InputMode::Gripper || !h.startup_open,
                "Gripper input requires startup_open: false (wait for a command)");
    }
}

Config load_config(const std::string& path) {
    Config config;
    if (path.empty()) return config;
    const auto root = YAML::LoadFile(path);
    keys(root, {"discovery", "period_ms", "hands"});
    read_unsigned(root, "period_ms", config.period_ms);
    if (const auto d = root["discovery"]) {
        keys(d, {"baudrate", "attempts", "probe_attempts", "settle_ms", "probe_retry_ms", "retry_ms", "require_both"});
        read_unsigned(d, "baudrate", config.discovery.baudrate);
        read_unsigned(d, "attempts", config.discovery.attempts);
        read_unsigned(d, "probe_attempts", config.discovery.probe_attempts);
        read_unsigned(d, "settle_ms", config.discovery.settle_ms);
        read_unsigned(d, "probe_retry_ms", config.discovery.probe_retry_ms);
        read_unsigned(d, "retry_ms", config.discovery.retry_ms);
        read(d, "require_both", config.discovery.require_both);
    }
    const auto hands = root["hands"];
    keys(hands, {"left", "right"});
    for (auto& h : config.hands) {
        if (!hands || !hands[h.name]) continue;
        const auto node = hands[h.name];
        keys(node, {"slave_id", "port", "command_topic", "input", "control", "duration_ms",
                    "command_timeout_ms", "startup_open", "gripper"});
        if (node["slave_id"]) {
            const int id = node["slave_id"].as<int>();
            require(id > 0 && id < 255, "slave_id must be in [1, 254]");
            h.slave_id = static_cast<uint8_t>(id);
        }
        read(node, "port", h.port);
        read(node, "command_topic", h.command_topic);
        if (node["input"]) {
            const auto mode = node["input"].as<std::string>();
            require(mode == "auto" || mode == "fingers" || mode == "gripper",
                    "input must be auto, fingers or gripper");
            h.input = mode == "auto" ? InputMode::Auto
                                    : mode == "fingers" ? InputMode::Fingers : InputMode::Gripper;
        }
        if (node["control"]) {
            const auto mode = node["control"].as<std::string>();
            require(mode == "position_speed" || mode == "position_time",
                    "control must be position_speed or position_time");
            h.control = mode == "position_speed" ? ControlMode::PositionSpeed : ControlMode::PositionTime;
        }
        // Auto preserves the legacy hand-level policy; VLA profiles opt into a watchdog.
        h.startup_open = h.input != InputMode::Gripper;
        h.command_timeout_ms = h.input == InputMode::Gripper ? 500 : 0;
        read(node, "startup_open", h.startup_open);
        read_unsigned(node, "command_timeout_ms", h.command_timeout_ms);
        if (node["duration_ms"]) {
            const int duration = node["duration_ms"].as<int>();
            require(duration > 0 && duration <= 2000, "duration_ms must be in [1, 2000]");
            h.duration_ms = static_cast<uint16_t>(duration);
        }
        if (const auto g = node["gripper"]) {
            keys(g, {"index", "open_value", "closed_value", "open_pose", "closed_pose", "speed"});
            unsigned index = 0;
            read_unsigned(g, "index", index);
            h.gripper.index = index;
            read(g, "open_value", h.gripper.open_value);
            read(g, "closed_value", h.gripper.closed_value);
            read(g, "speed", h.gripper.speed);
            pose(g, "open_pose", h.gripper.open_pose);
            pose(g, "closed_pose", h.gripper.closed_pose);
        }
    }
    validate_config(config);
    return config;
}

} // namespace brainco
