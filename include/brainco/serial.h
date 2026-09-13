#pragma once

#include <stark-sdk.h>
#include "brainco/command.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace brainco {

struct ModbusCloser {
    void operator()(DeviceHandler* handle) const { if (handle) modbus_close(handle); }
};
using ModbusHandle = std::unique_ptr<DeviceHandler, ModbusCloser>;

struct SerialBus {
    std::string port;
    ModbusHandle handle;
    std::vector<std::size_t> hands;
};

using Log = std::function<void(const std::string&)>;
using KeepRunning = std::function<bool()>;
using Wait = std::function<void(unsigned)>;

// One owned handle and one worker per physical port, including a shared RS485 bus.
// Supplying explicit ports also supports /dev/serial/by-id symlinks.
std::vector<std::string> serial_ports(const Config& config);
std::vector<SerialBus> discover(const Config& config, const KeepRunning& running,
                              const Wait& wait, const Log& log);
void send_command(DeviceHandler* handle, const HandConfig& config, const HandCommand& command);
void stop_motion(DeviceHandler* handle, uint8_t slave_id);

} // namespace brainco
