#include "brainco/command.h"
#include <atomic>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)

template<typename Function>
void rejects(Function function) {
    bool rejected = false;
    try { function(); } catch (const std::exception&) { rejected = true; }
    CHECK(rejected);
}

void test_auto_input(const std::string& root) {
    using namespace brainco;
    auto config = load_config(root + "/config/gripper.yaml");
    for (const auto& hand : config.hands) {
        CHECK(hand.input == InputMode::Auto);
        CHECK(hand.command_topic == "rt/brainco/" + hand.name + "/cmd");
        CHECK(!hand.startup_open && hand.command_timeout_ms == 500);
        const auto gripper = retarget(hand, {{0.5, std::numeric_limits<double>::quiet_NaN()}});
        CHECK(gripper); // Scalar dq is unused; speed comes from gripper configuration.
        for (std::size_t i = 0; i < finger_count; ++i) {
            CHECK(gripper->positions[i] == 500 && gripper->speeds[i] == 300);
        }
        for (std::size_t size = 0; size <= 8; ++size) {
            if (size != 1 && size != finger_count) CHECK(!retarget(hand, std::vector<MotorInput>(size)));
        }
        CHECK(!retarget(hand, {{std::numeric_limits<double>::quiet_NaN(), 0}}));
        CHECK(!retarget(hand, {{std::numeric_limits<double>::infinity(), 0}}));
    }

    auto& left = config.hands[0];
    auto& right = config.hands[1];
    left.control = ControlMode::PositionTime;
    right.gripper.open_value = 1;
    right.gripper.closed_value = 0;
    validate_config(config);
    CommandBuffer left_buffer(left), right_buffer(right);
    CHECK(!left_buffer.snapshot() && !right_buffer.snapshot());
    const auto now = Clock::now();
    CHECK(left_buffer.accept({{0.25, 0}}, now));
    CHECK(right_buffer.accept({{0.25, 0}}, now));
    CHECK(left_buffer.snapshot()->command.positions[0] == 250);
    CHECK(right_buffer.snapshot()->command.positions[0] == 750); // Independent calibration.

    CommandSchedule schedule;
    CHECK(schedule.next(left, left_buffer.snapshot(), now) == CommandAction::Send);
    CHECK(schedule.next(left, left_buffer.snapshot(), now + std::chrono::milliseconds(10)) == CommandAction::None);
    const auto previous = left_buffer.snapshot();
    CHECK(!left_buffer.accept(std::vector<MotorInput>(2), now + std::chrono::milliseconds(400)));
    CHECK(left_buffer.snapshot()->sequence == previous->sequence);
    CHECK(left_buffer.snapshot()->received == now);
    CHECK(schedule.next(left, left_buffer.snapshot(), now + std::chrono::milliseconds(500)) == CommandAction::Stop);

    // Six entries with five zero positions still mean finger control, not a gripper.
    std::vector<MotorInput> fingers(6);
    fingers[0] = {0.7f, 0.9f};
    CHECK(left_buffer.accept(fingers, now + std::chrono::milliseconds(600)));
    const auto native = left_buffer.snapshot();
    CHECK(native->command.positions[0] == 700);
    for (std::size_t i = 1; i < finger_count; ++i) CHECK(native->command.positions[i] == 0);
    CHECK(schedule.next(left, native, now + std::chrono::milliseconds(600)) == CommandAction::Send);
    CHECK(schedule.next(left, native, now + std::chrono::milliseconds(610)) == CommandAction::None);
    CHECK(right_buffer.snapshot()->command.positions[0] == 750);

    // Format selection happens on every sample and does not become locked to a hand.
    CHECK(left_buffer.accept({{0.5, 0}}, now + std::chrono::milliseconds(620)));
    CHECK(schedule.next(left, left_buffer.snapshot(), now + std::chrono::milliseconds(620)) == CommandAction::Send);
    const auto remapped = left_buffer.snapshot();
    for (const auto value : remapped->command.positions) CHECK(value == 500);
    CHECK(right_buffer.accept(fingers, now + std::chrono::milliseconds(620)));
    CHECK(right_buffer.snapshot()->command.positions[0] == 700);
    CHECK(right_buffer.snapshot()->command.positions[1] == 0);
    CHECK(right_buffer.snapshot()->command.speeds[0] == 900);

    const auto defaults = load_config("");
    CHECK(defaults.hands[0].input == InputMode::Auto && defaults.hands[1].input == InputMode::Auto);
    CHECK(defaults.hands[0].startup_open && defaults.hands[0].command_timeout_ms == 0);
    const auto fixed = load_config(root + "/test/input_modes.yaml");
    CHECK(fixed.hands[0].input == InputMode::Fingers && fixed.hands[0].startup_open);
    CHECK(fixed.hands[0].command_timeout_ms == 0);
    CHECK(!retarget(fixed.hands[0], {{0.5, 1}}));
    CHECK(retarget(fixed.hands[0], fingers)->positions[0] == 700);
    CHECK(fixed.hands[1].input == InputMode::Gripper && !fixed.hands[1].startup_open);
    CHECK(fixed.hands[1].command_timeout_ms == 500);
    fingers[1].q = 0.25;
    const auto forced_gripper = retarget(fixed.hands[1], fingers);
    CHECK(forced_gripper);
    for (const auto value : forced_gripper->positions) CHECK(value == 250);
    config.hands[0].gripper.index = 1;
    rejects([&] { validate_config(config); });
}

int main(int argc, char** argv) {
    try {
        CHECK(argc == 2);
        using namespace brainco;
        const std::string root = argv[1];
        test_auto_input(root);
        auto config = load_config(root + "/config/dual_revo2.yaml");
        CHECK(config.hands[0].input == InputMode::Auto && config.hands[1].input == InputMode::Auto);
        CHECK(config.hands[0].slave_id == 126 && config.hands[1].slave_id == 127);
        CHECK(config.hands[0].startup_open && config.hands[0].command_timeout_ms == 0);
        auto hand = config.hands[0];
        auto result = retarget(hand, {{-1, -1}, {0.25, 0.5}, {2, 2}, {1, 0}, {0, 1}, {0.75, 0.2}});
        CHECK(result && (result->positions == std::array<uint16_t, 6>{0, 250, 1000, 1000, 0, 750}));
        CHECK((result->speeds == std::array<uint16_t, 6>{0, 500, 1000, 0, 1000, 200}));
        CHECK(retarget(hand, std::vector<MotorInput>(6, {0.7f, 0.9f}))->positions[0] == 700);
        CHECK(!retarget(hand, {}));
        CHECK(!retarget(hand, std::vector<MotorInput>(5)));
        CHECK(!retarget(hand, std::vector<MotorInput>(7)));
        std::vector<MotorInput> invalid(6);
        invalid[2].q = std::numeric_limits<double>::quiet_NaN();
        CHECK(!retarget(hand, invalid));
        invalid[2] = {0, std::numeric_limits<double>::infinity()};
        CHECK(!retarget(hand, invalid));
        hand.control = ControlMode::PositionTime;
        CHECK(retarget(hand, invalid)); // dq is unused in duration mode.
        CHECK(!load_config("").discovery.require_both); // Bare launch retains partial-hand policy.

        auto grippers = load_config(root + "/config/gripper.yaml");
        hand = grippers.hands[0];
        CHECK(!hand.startup_open && hand.command_timeout_ms == 500);
        hand.input = InputMode::Gripper; // Explicit mode supports arbitrary scalar indices.
        hand.gripper.index = 1;
        hand.gripper.open_value = 0.08; // width in metres: larger = open
        hand.gripper.closed_value = 0;
        hand.gripper.open_pose = {0, 0.7, 0.1, 0.2, 0.3, 0.4};
        hand.gripper.closed_pose = {1, 0.7, 0.9, 0.8, 0.7, 0.6};
        result = retarget(hand, {{99, 99}, {0.04, 0}});
        CHECK(result && result->positions[0] == 500 && result->positions[1] == 700);
        for (std::size_t i = 2; i < 6; ++i) CHECK(result->positions[i] >= 499 && result->positions[i] <= 500);
        CHECK(result->speeds[0] == 300);
        CHECK(retarget(hand, {{}, {0.1, 0}})->positions[0] == 0);
        CHECK(retarget(hand, {{}, {-1, 0}})->positions[0] == 1000);
        CHECK(!retarget(hand, {{0, 0}}));
        CHECK(!retarget(hand, {{}, {std::numeric_limits<double>::infinity(), 0}}));

        // Reject malformed input without replacing the target or refreshing its age.
        CommandBuffer buffer(hand);
        CHECK(!buffer.snapshot());
        const auto now = Clock::now();
        CHECK(buffer.accept({{}, {0.04, 0}}, now));
        const auto sample = buffer.snapshot();
        CHECK(sample && !buffer.accept({}, now + std::chrono::milliseconds(400)));
        CHECK(buffer.snapshot()->sequence == sample->sequence);
        CHECK(buffer.snapshot()->received == now);
        CommandSchedule schedule;
        CHECK(schedule.next(hand, sample, now) == CommandAction::Send);
        CHECK(schedule.next(hand, sample, now + std::chrono::milliseconds(499)) == CommandAction::Send);
        CHECK(schedule.next(hand, sample, now + std::chrono::milliseconds(500)) == CommandAction::Stop);
        CHECK(schedule.next(hand, sample, now + std::chrono::seconds(2)) == CommandAction::None);
        CHECK(buffer.accept({{}, {0, 0}}, now + std::chrono::seconds(2)));
        CHECK(schedule.next(hand, buffer.snapshot(), now + std::chrono::seconds(2)) == CommandAction::Send);

        hand.control = ControlMode::PositionTime;
        CommandSchedule timed;
        CHECK(timed.next(hand, sample, now) == CommandAction::Send);
        CHECK(timed.next(hand, sample, now + std::chrono::milliseconds(10)) == CommandAction::None);
        CHECK(timed.next(hand, sample, now + std::chrono::milliseconds(500)) == CommandAction::Stop);
        CHECK(timed.next(hand, buffer.snapshot(), now + std::chrono::seconds(2)) == CommandAction::Send);

        CommandBuffer legacy(config.hands[0]);
        CHECK(legacy.snapshot()->command.positions[0] == 0);
        CHECK(legacy.snapshot()->command.speeds[0] == 1000);
        CommandSchedule legacy_schedule;
        CHECK(legacy_schedule.next(config.hands[0], legacy.snapshot(), now + std::chrono::hours(1)) == CommandAction::Send);

        // Concurrent readers must see complete six-finger samples.
        std::atomic<bool> complete{false};
        std::thread writer([&] {
            for (int i = 0; i < 20000; ++i)
                legacy.accept(std::vector<MotorInput>(i % 2 ? 1 : 6, {double(i % 2), 1}));
            complete = true;
        });
        bool coherent = true;
        while (!complete) {
            const auto current = legacy.snapshot();
            for (const auto position : current->command.positions)
                coherent &= position == current->command.positions[0];
        }
        writer.join();
        CHECK(coherent);

        config.discovery.attempts = 0;
        rejects([&] { validate_config(config); });
        config = Config{};
        config.hands[1].slave_id = config.hands[0].slave_id;
        rejects([&] { validate_config(config); });
        config = grippers;
        config.hands[0].gripper.closed_value = config.hands[0].gripper.open_value;
        rejects([&] { validate_config(config); });
        config = grippers;
        config.hands[0].gripper.speed = 0;
        rejects([&] { validate_config(config); });
        config = grippers;
        config.hands[0].gripper.closed_pose[0] = std::numeric_limits<double>::quiet_NaN();
        rejects([&] { validate_config(config); });
        rejects([&] { load_config(root + "/test/invalid_config.yaml"); });
        std::cout << "Automatic/native/gripper mapping, configuration, watchdog and concurrent snapshot tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
