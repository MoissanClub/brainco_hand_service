#include "brainco/serial.h"
#include <algorithm>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace brainco {
namespace {

std::string canonical_port(const std::string& port) {
    std::error_code error;
    const auto canonical = std::filesystem::canonical(port, error);
    return error ? port : canonical.string();
}

bool is_revo2(StarkHardwareType type) {
    // The enum values changed in SDK 2.0. Use the selected header's names.
    switch (type) {
        case STARK_HARDWARE_TYPE_REVO2_BASIC:
        case STARK_HARDWARE_TYPE_REVO2_TOUCH:
        case STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE:
#ifdef BRAINCO_SDK_HAS_EXTENDED_REVO2
        case STARK_HARDWARE_TYPE_REVO2_TOUCH_FORCE3D:
        case STARK_HARDWARE_TYPE_REVO2_TOUCH_ARRAY_PRESSURE:
#endif
            return true;
        default: return false;
    }
}

bool matches_side(SkuType sku, const std::string& side) {
    return side == "left" ? sku == SKU_TYPE_MEDIUM_LEFT || sku == SKU_TYPE_SMALL_LEFT
                          : sku == SKU_TYPE_MEDIUM_RIGHT || sku == SKU_TYPE_SMALL_RIGHT;
}

bool probe(DeviceHandler* handle, const HandConfig& hand, const DiscoveryConfig& config,
           const KeepRunning& running, const Wait& wait, const Log& log) {
    for (unsigned attempt = 0; attempt < config.probe_attempts && running(); ++attempt) {
        if (attempt) wait(config.probe_retry_ms);
        if (!running()) return false;
        uint16_t value = 0;
        if (stark_read_input_registers(handle, hand.slave_id, 3000, 1, &value) != 0) continue;
        std::unique_ptr<CDeviceInfo, decltype(&free_device_info)> info(
            stark_get_device_info(handle, hand.slave_id), free_device_info);
        if (!info) continue;
        if (!is_revo2(info->hardware_type) || !matches_side(info->sku_type, hand.name)) {
            log("Rejected slave " + std::to_string(hand.slave_id) + ": hardware=" +
                std::to_string(static_cast<unsigned>(info->hardware_type)) + " sku=" +
                std::to_string(static_cast<unsigned>(info->sku_type)) + " (expected Revo2 " + hand.name + ")");
            return false;
        }
        // Only configure a device after validating both generation and side.
        stark_set_finger_unit_mode(handle, hand.slave_id, FINGER_UNIT_MODE_NORMALIZED);
        // The SDK getter returns Normalized on failure. Use raw readback instead.
        if (stark_read_holding_registers(handle, hand.slave_id, 937, 1, &value) != 0 || value != 0)
            continue;
        log(hand.name + " Revo2: firmware=" + (info->firmware_version ? info->firmware_version : "unknown") +
            " serial=" + (info->serial_number ? info->serial_number : "unknown"));
        return true;
    }
    return false;
}

} // namespace

std::vector<std::string> serial_ports(const Config& config) {
    std::set<std::string> ports;
    bool scan = false;
    for (const auto& hand : config.hands) {
        if (hand.port.empty()) scan = true;
        else ports.insert(canonical_port(hand.port));
    }
    if (scan) {
        for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
            const auto name = entry.path().filename().string();
            if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyHAND", 0) == 0 || name.rfind("ttyUN", 0) == 0)
                ports.insert(canonical_port(entry.path().string()));
        }
    }
    return {ports.begin(), ports.end()};
}

std::vector<SerialBus> discover(const Config& config, const KeepRunning& running,
                              const Wait& wait, const Log& log) {
    std::vector<SerialBus> buses;
    std::array<bool, 2> found{};
    const auto& d = config.discovery;
    for (unsigned attempt = 0; attempt < d.attempts && running(); ++attempt) {
        if (attempt) wait(d.retry_ms);
        if (!running()) break;
        log("Discovery pass " + std::to_string(attempt + 1) + "/" + std::to_string(d.attempts));
        // Refresh every pass so a USB adapter enumerating late is picked up.
        for (const auto& port : serial_ports(config)) {
            if (!running()) break;
            std::vector<std::size_t> candidates;
            for (std::size_t i = 0; i < config.hands.size(); ++i) {
                const auto& hand = config.hands[i];
                if (!found[i] && (hand.port.empty() || canonical_port(hand.port) == port)) candidates.push_back(i);
            }
            if (candidates.empty()) continue;
            auto existing = std::find_if(buses.begin(), buses.end(), [&](const auto& bus) { return bus.port == port; });
            SerialBus opened{port, ModbusHandle{}, {}};
            SerialBus* bus = nullptr;
            if (existing != buses.end()) bus = &*existing;
            else {
                opened.handle.reset(modbus_open(port.c_str(), d.baudrate));
                if (!opened.handle) {
                    log("Cannot open " + port + " at " + std::to_string(d.baudrate) + " baud; will retry");
                    continue;
                }
                // Same cold-open mitigation as SDK c/demo/debug_detect.cpp.
                wait(d.settle_ms);
                bus = &opened;
            }
            for (std::size_t i : candidates) {
                if (!running()) break;
                const auto& hand = config.hands[i];
                if (!probe(bus->handle.get(), hand, d, running, wait, log)) continue;
                found[i] = true;
                bus->hands.push_back(i);
                log(hand.name + " hand bound to " + port + " port (slave " + std::to_string(hand.slave_id) + ")");
            }
            if (bus == &opened && !opened.hands.empty()) buses.push_back(std::move(opened));
            if (found[0] && found[1]) return buses;
        }
    }
    if (!running()) return {};
    for (std::size_t i = 0; i < found.size(); ++i)
        if (!found[i]) log("Missing " + config.hands[i].name + " Revo2 hand after discovery retries");
    if ((d.require_both && (!found[0] || !found[1])) || buses.empty())
        throw std::runtime_error("Hand discovery incomplete. Check hand power/calibration, serial permissions, "
                                 "other port users, baudrate, slave IDs and configured ports.");
    return buses;
}

void send_command(DeviceHandler* handle, const HandConfig& config, const HandCommand& command) {
    if (config.control == ControlMode::PositionSpeed) {
        stark_set_finger_positions_and_speeds(handle, config.slave_id, command.positions.data(),
                                             command.speeds.data(), finger_count);
    } else {
        std::array<uint16_t, finger_count> durations;
        durations.fill(config.duration_ms);
        stark_set_finger_positions_and_durations(handle, config.slave_id, command.positions.data(),
                                                durations.data(), finger_count);
    }
}

void stop_motion(DeviceHandler* handle, uint8_t slave_id) {
    const std::array<int16_t, finger_count> stopped{};
    stark_set_finger_speeds(handle, slave_id, stopped.data(), stopped.size());
}

} // namespace brainco
