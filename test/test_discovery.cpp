#include "brainco/serial.h"
#include <iostream>
#include <map>
#include <stdexcept>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)

struct DeviceHandler { std::string port; unsigned opened_at; };
struct Device {
    StarkHardwareType hardware = STARK_HARDWARE_TYPE_REVO2_BASIC;
    SkuType sku = SKU_TYPE_MEDIUM_LEFT;
    unsigned failed_pings = 0;
    unsigned failed_infos = 0;
    unsigned failed_readbacks = 0;
};
std::map<std::pair<std::string, uint8_t>, Device> devices;
std::map<std::string, unsigned> opens, failed_opens;
unsigned tick = 0, closes = 0, active = 0, infos = 0, freed_infos = 0, configurations = 0;
unsigned speed_commands = 0, time_commands = 0, stops = 0;
uint8_t last_slave = 0;
uint16_t last_position = 0, last_parameter = 0;

extern "C" {
DeviceHandler* modbus_open(const char* port, uint32_t baudrate) {
    CHECK(baudrate == 460800);
    ++opens[port];
    if (failed_opens[port]) { --failed_opens[port]; return nullptr; }
    ++active;
    return new DeviceHandler{port, tick};
}
void modbus_close(DeviceHandler* handle) { ++closes; --active; delete handle; }
int32_t stark_read_input_registers(DeviceHandler* handle, uint8_t slave, uint16_t address,
                                  uint16_t count, uint16_t* output) {
    CHECK(tick - handle->opened_at >= 150); // Every new open must settle before probing.
    CHECK(address == 3000 && count == 1);
    const auto it = devices.find({handle->port, slave});
    if (it == devices.end()) return -1;
    if (it->second.failed_pings) { --it->second.failed_pings; return -1; }
    *output = 1;
    return 0;
}
CDeviceInfo* stark_get_device_info(DeviceHandler* handle, uint8_t slave) {
    auto& device = devices.at({handle->port, slave});
    if (device.failed_infos) { --device.failed_infos; return nullptr; }
    auto* info = new CDeviceInfo{};
    info->hardware_type = device.hardware;
    info->sku_type = device.sku;
    // Exercise null strings too.
    ++infos;
    return info;
}
void free_device_info(CDeviceInfo* info) { ++freed_infos; delete info; }
void stark_set_finger_unit_mode(DeviceHandler* handle, uint8_t slave, FingerUnitMode mode) {
    CHECK(mode == FINGER_UNIT_MODE_NORMALIZED);
    CHECK(devices.at({handle->port, slave}).hardware != STARK_HARDWARE_TYPE_REVO1_ADVANCED);
    ++configurations;
}
int32_t stark_read_holding_registers(DeviceHandler* handle, uint8_t slave, uint16_t address,
                                    uint16_t count, uint16_t* output) {
    CHECK(address == 937 && count == 1);
    auto& device = devices.at({handle->port, slave});
    if (device.failed_readbacks) { --device.failed_readbacks; return -1; }
    *output = 0;
    return 0;
}
void stark_set_finger_positions_and_speeds(DeviceHandler*, uint8_t slave, const uint16_t* positions,
                                         const uint16_t* speeds, uintptr_t count) {
    CHECK(count == 6);
    ++speed_commands; last_slave = slave; last_position = positions[0]; last_parameter = speeds[0];
}
void stark_set_finger_positions_and_durations(DeviceHandler*, uint8_t slave, const uint16_t* positions,
                                            const uint16_t* durations, uintptr_t count) {
    CHECK(count == 6);
    ++time_commands; last_slave = slave; last_position = positions[0]; last_parameter = durations[0];
}
void stark_set_finger_speeds(DeviceHandler*, uint8_t slave, const int16_t* speeds, uintptr_t count) {
    CHECK(count == 6);
    for (std::size_t i = 0; i < count; ++i) CHECK(speeds[i] == 0);
    ++stops; last_slave = slave;
}
}

void reset() {
    CHECK(active == 0 && infos == freed_infos);
    devices.clear(); opens.clear(); failed_opens.clear();
    tick = closes = infos = freed_infos = configurations = 0;
}

int main() {
    try {
        using namespace brainco;
        const auto wait = [](unsigned millis) { tick += millis; };
        const auto log = [](const std::string&) {};
        const auto running = [] { return true; };
        Config config;
        config.discovery.require_both = true;
        config.hands[0].port = "/mock/z-left";
        config.hands[1].port = "/mock/a-right";
        devices[{"/mock/z-left", 126}] = {STARK_HARDWARE_TYPE_REVO2_BASIC, SKU_TYPE_MEDIUM_LEFT, 1};
        devices[{"/mock/a-right", 127}] = {STARK_HARDWARE_TYPE_REVO2_TOUCH, SKU_TYPE_MEDIUM_RIGHT, 1, 1};
        {
            auto buses = discover(config, running, wait, log);
            CHECK(buses.size() == 2);
            CHECK(buses[0].hands[0] == 1 && buses[1].hands[0] == 0); // Right enumerated first.
            CHECK(opens["/mock/z-left"] == 1 && opens["/mock/a-right"] == 1);
        }
        CHECK(closes == 2);
        reset();

        // Left stays connected while right requires a complete close/reopen retry.
        devices[{"/mock/z-left", 126}] = {STARK_HARDWARE_TYPE_REVO2_BASIC, SKU_TYPE_MEDIUM_LEFT};
        devices[{"/mock/a-right", 127}] = {STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE, SKU_TYPE_MEDIUM_RIGHT, 3};
#ifdef BRAINCO_SDK_HAS_EXTENDED_REVO2
        devices.at({"/mock/z-left", 126}).hardware = STARK_HARDWARE_TYPE_REVO2_TOUCH_FORCE3D;
        devices.at({"/mock/a-right", 127}).hardware = STARK_HARDWARE_TYPE_REVO2_TOUCH_ARRAY_PRESSURE;
#endif
        failed_opens["/mock/a-right"] = 1;
        {
            auto buses = discover(config, running, wait, log);
            CHECK(buses.size() == 2 && opens["/mock/z-left"] == 1 && opens["/mock/a-right"] == 3);
        }
        reset();

        config.hands[0].port = config.hands[1].port = "/mock/shared";
        devices[{"/mock/shared", 126}] = {STARK_HARDWARE_TYPE_REVO2_BASIC, SKU_TYPE_MEDIUM_LEFT};
        devices[{"/mock/shared", 127}] = {STARK_HARDWARE_TYPE_REVO2_BASIC, SKU_TYPE_MEDIUM_RIGHT, 3, 0, 1};
        {
            auto buses = discover(config, running, wait, log);
            CHECK(buses.size() == 1 && buses[0].hands.size() == 2);
            CHECK(opens["/mock/shared"] == 1); // Never open two handles for one bus.
            HandCommand command;
            command.positions.fill(250); command.speeds.fill(300);
            send_command(buses[0].handle.get(), config.hands[0], command);
            CHECK(speed_commands == 1 && last_slave == 126 && last_position == 250 && last_parameter == 300);
            config.hands[1].control = ControlMode::PositionTime;
            config.hands[1].duration_ms = 120;
            send_command(buses[0].handle.get(), config.hands[1], command);
            CHECK(time_commands == 1 && last_slave == 127 && last_parameter == 120);
            stop_motion(buses[0].handle.get(), 127);
            CHECK(stops == 1);
        }
        reset();

        // Reject Revo1 Advanced even though it shares Revo2's motor API and IDs.
        devices[{"/mock/shared", 126}] = {STARK_HARDWARE_TYPE_REVO1_ADVANCED, SKU_TYPE_MEDIUM_LEFT};
        devices[{"/mock/shared", 127}] = {STARK_HARDWARE_TYPE_REVO2_BASIC, SKU_TYPE_MEDIUM_LEFT}; // Wrong side.
        bool failed = false;
        try { discover(config, running, wait, log); } catch (const std::runtime_error&) { failed = true; }
        CHECK(failed && configurations == 0 && opens["/mock/shared"] == config.discovery.attempts);
        reset();

        // Partial discovery cleans up before reporting failure; explicit partial mode is supported.
        devices[{"/mock/shared", 126}] = {STARK_HARDWARE_TYPE_REVO2_BASIC, SKU_TYPE_MEDIUM_LEFT};
        failed = false;
        try { discover(config, running, wait, log); } catch (const std::runtime_error&) { failed = true; }
        CHECK(failed && active == 0);
        config.discovery.require_both = false;
        { auto buses = discover(config, running, wait, log); CHECK(buses.size() == 1 && buses[0].hands.size() == 1); }
        reset();

        CHECK(discover(config, [] { return false; }, wait, log).empty());
        CHECK(opens.empty());
        bool keep_running = true;
        CHECK(discover(config, [&] { return keep_running; },
                       [&](unsigned ms) { wait(ms); keep_running = false; }, log).empty());
        CHECK(active == 0 && infos == 0);
        reset();
        std::cout << "Cold-open, retries, dual-hand/shared-bus, validation, cleanup and control dispatch tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
