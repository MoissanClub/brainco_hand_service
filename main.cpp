#include "brainco/config.h"
#include "brainco/serial.h"
#include "dds/Publisher.h"
#include "param.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <atomic>
#include <csignal>
#include <thread>

namespace {
std::atomic<bool> stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free, "Signal handler requires lock-free atomics");
std::atomic<bool> worker_failed{false};
void signal_handler(int) { stop_requested.store(true, std::memory_order_relaxed); }
bool running() { return !stop_requested.load(std::memory_order_relaxed) && !worker_failed.load(); }

void interruptible_wait(unsigned millis) {
    const auto deadline = brainco::Clock::now() + std::chrono::milliseconds(millis);
    while (running() && brainco::Clock::now() < deadline)
        std::this_thread::sleep_until(std::min(deadline, brainco::Clock::now() + std::chrono::milliseconds(20)));
}

class HandBridge {
public:
    explicit HandBridge(const brainco::HandConfig& config)
        : config_(config), commands_(config), state_("rt/brainco/" + config.name + "/state"),
          subscriber_(config.command_topic) {
        // All callback state exists before InitChannel starts receiving.
        subscriber_.InitChannel([this](const void* data) {
            const auto& message = *static_cast<const unitree_go::msg::dds_::MotorCmds_*>(data);
            std::vector<brainco::MotorInput> input;
            input.reserve(message.cmds().size());
            for (const auto& motor : message.cmds()) input.push_back({motor.q(), motor.dq()});
            if (!commands_.accept(input)) {
                if (++invalid_commands_ == 1 || invalid_commands_ % 100 == 0)
                    spdlog::warn("{}: rejected malformed/nonfinite command on {} (count {})",
                                 config_.name, config_.command_topic, invalid_commands_);
            }
        }, 1);
        spdlog::info("Starting worker for {} (slave {}), input topic {}", config.name,
                     static_cast<unsigned>(config.slave_id), config.command_topic);
    }

    ~HandBridge() { subscriber_.CloseChannel(); }

    void update(DeviceHandler* handle) {
        const auto sample = commands_.snapshot();
        switch (schedule_.next(config_, sample)) {
            case brainco::CommandAction::Send:
                brainco::send_command(handle, config_, sample->command);
                break;
            case brainco::CommandAction::Stop:
                brainco::stop_motion(handle, config_.slave_id);
                spdlog::warn("{} command timed out; sent zero velocity", config_.name);
                break;
            case brainco::CommandAction::None: break;
        }

        std::unique_ptr<CMotorStatusData, decltype(&free_motor_status_data)> status(
            stark_get_motor_status(handle, config_.slave_id), free_motor_status_data);
        if (!status) {
            if (++status_failures_ == 1 || status_failures_ % 100 == 0)
                spdlog::warn("{} motor status read failed ({} consecutive failures)", config_.name, status_failures_);
            return;
        }
        if (status_failures_) spdlog::info("{} status communication recovered", config_.name);
        status_failures_ = 0;
        if (!state_.trylock()) return;
        auto& states = state_.msg_.states();
        states.resize(brainco::finger_count);
        for (std::size_t i = 0; i < brainco::finger_count; ++i) {
            states[i].q() = status->positions[i] / 1000.f;
            states[i].dq() = status->speeds[i] / 1000.f;
            states[i].tau_est() = status->currents[i] / 1000.f;
        }
        state_.unlockAndPublish();
    }

private:
    const brainco::HandConfig config_;
    brainco::CommandBuffer commands_;
    brainco::CommandSchedule schedule_;
    unsigned status_failures_ = 0;
    unsigned invalid_commands_ = 0;
    unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_> state_;
    // Destroy the subscriber before callback state.
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorCmds_> subscriber_;
};

void bus_worker(brainco::SerialBus& bus, const brainco::Config& config) {
    try {
        std::vector<std::unique_ptr<HandBridge>> hands;
        for (const auto index : bus.hands) hands.push_back(std::make_unique<HandBridge>(config.hands[index]));
        while (running()) {
            const auto deadline = brainco::Clock::now() + std::chrono::milliseconds(config.period_ms);
            for (auto& hand : hands) {
                if (!running()) break;
                hand->update(bus.handle.get());
            }
            // Serial transactions determine the achievable rate; never catch up with a burst.
            if (running()) std::this_thread::sleep_until(deadline);
        }
    } catch (const std::exception& error) {
        spdlog::error("Worker on {} failed: {}", bus.port, error.what());
        worker_failed = true;
    }
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    try {
        const auto vm = param::helper(argc, argv);
        const auto config = brainco::load_config(vm["config"].as<std::string>());
        init_logging(LOG_LEVEL_ERROR);
        auto buses = brainco::discover(config, running, interruptible_wait,
                                      [](const auto& message) { spdlog::info("{}", message); });
        if (!running()) return 0;
        if (vm.count("detect-only")) return 0;
        unitree::robot::ChannelFactory::Instance()->Init(0, vm["network_interface"].as<std::string>());
        std::vector<std::thread> workers;
        try {
            for (auto& bus : buses) workers.emplace_back(bus_worker, std::ref(bus), std::cref(config));
        } catch (...) {
            worker_failed = true;
            for (auto& worker : workers) worker.join();
            throw;
        }
        for (auto& worker : workers) worker.join();
        // Buses own their handles; close them after every worker has stopped.
        return worker_failed ? 1 : 0;
    } catch (const std::exception& error) {
        spdlog::error("{}", error.what());
        return 1;
    }
}
