# Revo2 discovery, control and gripper mapping

The service supports Revo2 hands over Modbus/RS485. It separates serial discovery
and SDK writes (`src/serial.cpp`), validated configuration (`src/config.cpp`),
DDS-independent command mapping and scheduling (`include/brainco/command.h`),
and DDS/worker lifetime (`main.cpp`).

## Build and run

The default build uses the bundled SDK **v2.0.5**, with Linux amd64 (x86_64) and
arm64 (aarch64) libraries. CMake selects the library for the target architecture:

```bash
cd brainco_hand_service
cmake -S . -B build -DBRAINCO_SDK_ROOT=
cmake --build build -j4
ctest --test-dir build --output-on-failure
./bin/brainco_hand_server -n eth0 -c config/dual_revo2.yaml
```

Use a serial-permitted account (or the existing sudo launch). The build requires
Unitree SDK2, Boost program_options, yaml-cpp, spdlog and their dependencies.
Set `CMAKE_PREFIX_PATH` if these are installed outside standard paths.
To return an existing build to the bundled SDK, pass `-DBRAINCO_SDK_ROOT=`.
An external SDK is still supported: pass
`-DBRAINCO_SDK_ROOT=../brainco-hand-sdk/dist` to select the matching header and
host-architecture library downloaded by the sibling SDK's script.

The vendor libraries lack a SONAME. CMake marks that explicitly so the executable
resolves the selected SDK through its runtime search path, without depending on
the launch directory. The original `cd bin` launch also continues to work; use
`-c ../config/dual_revo2.yaml` when launching there.

Never replace only the SDK header or only the library: Revo2 hardware enum values
changed from 5/6/7 in v1.1.9 to 10/11/12 in v2.0.5, which also adds two Revo2
sensor variants. CMake selects a pair, prints both paths, and enables the new
Revo2 variants when present in that header. Runtime library overrides such as
`LD_LIBRARY_PATH` must also use the selected version and architecture.

### Updating the bundled SDK

The repo's `download-lib.sh` is adapted from BrainCo's SDK download script. It
runs only on Linux amd64/x86_64 or arm64/aarch64 and always updates **both**
architectures together, even when run on the G1. This keeps the shared header
consistent with both binaries; it does not install anything system-wide.

Stop the bridge and any concurrent builds before updating. From the repo root:

```bash
# Downloader dependencies on Ubuntu/Debian (in addition to normal shell tools).
sudo apt install curl unzip binutils util-linux
./download-lib.sh                 # Pinned default: v2.0.5
# Or select a release explicitly: ./download-lib.sh --version v2.0.5
sha256sum -c lib/SHA256SUMS
cmake -S . -B build -DBRAINCO_SDK_ROOT=
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

The updater downloads `linux.zip` and `linux-arm64.zip` from BrainCo's HTTPS
release endpoint, checks ELF architectures and the bridge's required SDK
symbols, and requires identical headers in both archives. Only after both pass
does it replace `include/stark-sdk.h`, `lib/x86_64/libbc_stark_sdk.so`, and
`lib/aarch64/libbc_stark_sdk.so`, recording release URLs in `lib/VERSION` and
installed-file hashes in `lib/SHA256SUMS`. These hashes detect subsequent local
changes; they are not independent vendor signatures or an ABI compatibility
guarantee. Download/validation failures leave the installed SDK untouched;
installation errors and handled interrupts trigger backup restoration. The
multi-file update is not power-loss atomic. Re-running a release repairs missing
or changed files instead of trusting an existing version marker.

Keep the header, both binaries, version record and checksums together in version
control. Review SDK release notes and rebuild after every upgrade; downloading
successfully does not establish compatibility with the installed hand firmware.

## Startup discovery

The former service opened a port and immediately queried device info, tried each
combination once, and never rescanned. The SDK's
`c/demo/debug_detect.cpp` documents a cold-open USB/RS485 settling issue and waits
150 ms after `modbus_open`. This is consistent with the reported symptom, but
confirming the cause and tuning the delay require cold boots on the G1.

The new discovery path:

1. Enumerates and sorts `ttyUSB*`, `ttyHAND*` and `ttyUN*` ports; resolves symlinks
   to avoid opening an alias twice. Explicit per-hand `port` settings restrict
   scanning and accept stable `/dev/serial/by-id/...` paths.
2. Opens each candidate once, waits `settle_ms` (150 by default), and probes the
   configured IDs (left 126, right 127) at `baudrate` (460800).
3. Uses a short raw read of input register 3000, then reads full device identity.
   Each hand gets up to three probes separated by 100 ms.
4. Accepts only Revo2 hardware with the correct left/right SKU. Revo1 Advanced is
   rejected even though it shares the Revo2 motor API. Sets normalized units
   only after validation, and verifies register 937 with a read that can report
   failure; the SDK unit-mode getter returns Normalized even on failure.
5. Retains successful handles and rescans for missing hands up to five passes,
   500 ms apart. Unclaimed ports are closed and reopened on later passes;
   ports are enumerated again for USB devices that appear late.

Both supplied configurations set `discovery.require_both: true`. In that mode,
incomplete discovery exits with a nonzero status and releases every handle, so
a supervisor can retry. Without a config, or with `require_both: false`, the
service retains the former partial-hand startup policy after trying all passes.
Each physical port has one worker and one owning handle: independent adapters
run concurrently, while two IDs on one RS485 bus are serviced serially. Workers
start only after discovery completes. No discovery operation commands motion.

The retry counts bound application attempts, not total wall time: blocking SDK
reads have their own timeouts. SIGINT/SIGTERM interrupt the inter-probe waits;
an in-progress SDK call must finish. Power-on finger calibration still needs to
finish before normal control, as described in the
[Revo2 parameter documentation](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/parameters.html#description-of-the-power-on-auto-calibration-process).
The service does not initiate calibration or change its settings.

### Why not `stark_auto_detect` by default?

The downloaded header exposes
`stark_auto_detect(true, port, STARK_PROTOCOL_TYPE_MODBUS)`. `true` is essential
for finding all devices; the SDK's interactive `auto_detect_and_init` helper
actually passes `false` and initializes only one selected device. It cannot be
substituted directly for two-hand startup.

The general detector is useful when baudrate, IDs or transport are unknown. Its
documented Modbus search includes Revo1 IDs and multiple baudrates, and its API
does not expose settle/retry timing or a Revo2-only filter. Detection returns
metadata; `init_from_detected` then opens the serial transport again. Correct
integration would still need hardware/side filtering, retries, retained
connections, and one owner per physical port. An unrestricted AUTO scan also
examines CAN and legacy Protobuf transports.

For a G1 with known Revo2 RS485 settings, the targeted path gives explicit timing
and avoids the broader search. If a device uses a nondefault ID or baudrate,
configure it directly; no SDK auto-detection fallback is run silently. See the
local SDK header's detection declarations and
[BrainCo's C/C++ SDK reference](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/c_sdk.html).

## Compatibility and command behavior

Without a config file, the service defaults to `input: auto` on the existing
`rt/brainco/{left,right}/cmd` topics using `MotorCmds_`. It selects the adapter
independently for every received message on each hand:

| Number of `cmds` entries | Interpretation |
| --- | --- |
| 6 | Native per-finger `q`/`dq` commands, preserving the existing conversion. |
| 1 | Gripper scalar from `cmds[0].q()`, mapped to the configured six-finger pose. |
| Any other length | Rejected without changing the target or refreshing the timeout. |

A six-entry message with five zero positions is still a native finger command.
The bridge never guesses a format from the values, and accepts subsequent
messages in either format without a restart. Use one active command source per
hand; the newest valid received sample supplies the target regardless of format.
`input: fingers` enforces exactly six entries. `input: gripper` always reads the
configured scalar index, preserving support for other message lengths.

Feedback always remains six-finger `MotorStates_` on
`rt/brainco/{left,right}/state`, in the order
`[Thumb, Thumb_aux, Index, Middle, Ring, Pinky]`.

Position-speed control still clamps `q` and `dq` to [0,1], converts them to
[0,1000], and streams the last target at a nominal 100 Hz. The original float
conversion is preserved. Feedback `q`, signed `dq`, and signed `tau_est` still
divide the SDK's position, speed and current by 1000. **`tau_est` is the existing
normalized current proxy, not measured joint torque.** Actual rate depends on
serial latency, especially when two hands share a port.

The default auto/finger-input startup target remains fully open at maximum speed.
Set `startup_open: false` to wait for the first valid command. With
`command_timeout_ms: 0`, the last target persists indefinitely, as before.
In auto mode, startup and timeout settings apply to both formats; changing
message length does not change those settings. `config/gripper.yaml` sets
`startup_open: false` and a 500 ms timeout for either format. Nonfinite consumed
values are rejected as a whole. DDS callbacks and workers exchange locked
command snapshots; state publication acquires the publisher lock.

No-hand discovery now fails instead of returning success. Config errors also
fail before ports open. The dual-hand examples explicitly require both hands;
the bare launch retains partial-hand startup after retries.

## Choosing a Revo2 control mode

BrainCo describes five control modes. Recommendations below are engineering
choices for this bridge, based on its
[control-mode table](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/parameters.html#control-mode-description)
and [register definitions](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/modbus_pro.html).

| Mode | Use in this bridge / recommendation |
| --- | --- |
| Position + speed | Default `control: position_speed`; best starting point for streamed VLA targets and teleoperation. Position bounds travel while speed limits closure. Native input uses each `dq`; gripper input uses configured `gripper.speed`. |
| Position + time | Optional `control: position_time`, `duration_ms: 1..2000`; useful for discrete waypoints at a known cadence. Uses the SDK duration API once per new accepted DDS sample. `dq` is ignored. Repeated DDS samples still count as new waypoints, so choose their cadence and duration together. |
| Speed | Continuous signed motion until a limit or stall. Does not encode a gripper opening target; would need host position feedback and limits. Used here only to request zero velocity on command timeout. |
| Current | Continuous signed current target. Potentially useful for calibrated contact regulation, but current is not fingertip force. Needs application-specific feedback, limits and a grasp policy; not exposed as a generic VLA scalar mapping. |
| PWM | Continuous signed duty cycle. Requires a host controller and provides no position target; not exposed by this position bridge. |

In position-speed mode, **speed zero means maximum speed**, not stop. The legacy
`dq=0` interpretation is preserved. Gripper speed must be [0.001,1] to avoid
accidentally requesting maximum speed via zero. Position-time durations are
restricted to the SDK's documented positive range, excluding the firmware's
zero/max-speed special case. Mode selection uses the corresponding SDK command
function; DDS motor `mode`, `tau`, `kp`, and `kd` are not reinterpreted.

Turbo is a separate firmware stall/regrip behavior, not an additional position
controller. Leave it as an explicit application decision after grasp testing;
this bridge does not change turbo, protection-current or joint-limit settings.
For tactile Revo2 variants, contact/force feedback could later augment the
retargeter, but the current mapping is an open-loop pose interpolation.

## Current limitations and recommended next steps

The bridge exposes only part of the Revo2 SDK. This is a limitation of the
current message contract and serial worker, not of DDS as a transport. Updating
the bundled SDK does **not** automatically expose its additional APIs.

| Capability | Current bridge limitation |
| --- | --- |
| Movement speed | Native six-finger `dq=1` already requests 1000, the full normalized speed setting; actual speed depends on device limits and load. The gripper profile uses a fixed configured speed of 0.3. Zero speed in position-speed commands means maximum speed, not stop. |
| Control modes | Position-speed and position-time are selected in YAML, not per DDS command. Time mode uses one duration for all six fingers although the SDK accepts individual durations. Signed velocity, current and PWM commands are not exposed, except zero velocity for the watchdog. Incoming `mode`, `tau`, `kp`, and `kd` are ignored. |
| Feedback | Publishes six positions, speeds and currents only. `tau_est` is a normalized current proxy, not torque. SDK motor-state flags, tactile data and explicit communication-health/capability information are absent. Tactile availability depends on the Revo2 hardware variant. |
| Device settings | Speed/current/position limits, protection settings, turbo, calibration and stored gestures/action sequences are not configurable through DDS. |
| Timing and recovery | The nominal 100 Hz worker performs synchronous serial writes and reads. New commands replace older snapshots; there is no timestamped trajectory queue, execution acknowledgement or runtime port reconnection. Both hands sharing a bus share its bandwidth. |
| VLA retargeting | A scalar follows one configured open-to-closed pose curve, without contact feedback or grasp selection. Full six-finger commands remain available on the same topics. |

The SDK's motion/configuration functions and units are documented in
[BrainCo's C/C++ SDK reference](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/c_sdk.html)
and the [control-mode descriptions](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/parameters.html#control-mode-description).
Higher DDS publish rates do not imply faster physical movement. Measure achieved
serial throughput on the G1 before changing rates. The firmware's
[fast-position registers](https://staging.brainco.tech/docs/revolimb-hand/en/revo2/modbus_pro.html#fast-position-control-mode-1070-1072)
trade position resolution for a smaller command payload; they do not increase
the hand's mechanical speed limit.

Recommended order, preserving existing publishers:

1. **Version the command contract explicitly.** Keep the existing command topics
   and `MotorCmds_` type, with one/six entries still selecting gripper/finger
   representation. Define an opt-in version marker in `reserve` before assigning
   meanings to `mode` or other previously ignored fields. Unmarked messages must
   keep today's behavior. Start with runtime position-speed/position-time and
   per-finger durations, documented units and C++/Python publisher helpers. Do
   not silently attach a different DDS type to the existing topics.
2. **Add typed status and optional tactile topics** under the same
   `rt/brainco/{left,right}` namespace, retaining the legacy state topic. Expose
   explicit current units, motor flags, hardware/firmware capabilities, active
   mode, effective limits, sample age and communication failures. Use available
   tactile feedback to improve contact-aware grasping.
3. **Add acknowledged configuration requests** for limits and supported device
   features, outside the motion fast path. Serialize every SDK operation through
   the existing per-port owner. Where writes return `void`, use readback where
   available and distinguish DDS receipt from confirmed device state. Require
   explicit opt-in for calibration, turbo and direct current/PWM operation;
   these need application-specific limits and watchdogs.
4. **Measure and improve scheduling/recovery.** Record write/read latency and
   achieved rates before separating command, state and tactile polling rates.
   Add stale-state handling and reconnect with device identity/unit validation
   and an explicit resume policy. If VLA action chunks need timed execution,
   introduce a bounded timestamped queue with expiry and cancellation instead
   of publishing them rapidly into the current latest-sample buffer.

For VLA use, prioritize controllable closure speed/time, trustworthy feedback,
and validated limits/contact behavior before exposing low-level PWM.

## VLA gripper input

```bash
./bin/brainco_hand_server -n eth0 -c config/gripper.yaml
```

This profile uses `input: auto` on **`rt/brainco/left/cmd` and
`rt/brainco/right/cmd`**, with no separate VLA topics. Publish a Unitree
`MotorCmds_` containing exactly one entry per hand; its `q` is the gripper
scalar. By default, 0 means open and 1 means closed. Scalar `dq` is ignored;
closure speed comes from `gripper.speed`. Existing six-entry finger publishers
can use the same topics and continue supplying individual `q`/`dq` targets.
For example, a C++ publisher's message can be constructed as:

```cpp
unitree_go::msg::dds_::MotorCmds_ command;
command.cmds().resize(1);
command.cmds()[0].q() = 0.5f; // Halfway between the configured open/closed poses.
// Publish command on rt/brainco/left/cmd or rt/brainco/right/cmd.
```

Auto mode requires `gripper.index: 0`. For a publisher that packs both grippers
into one message, explicitly set `input: gripper` for each hand, set their
`command_topic` to that shared topic, and choose different indices (e.g. 0 and 1).
Other DDS types still need an adapter to supply the DDS-independent retargeter.

The mapping is:

```text
closure = clamp((q - open_value) / (closed_value - open_value), 0, 1)
finger[i] = open_pose[i] + closure * (closed_pose[i] - open_pose[i])
```

Set `open_value: 1` and `closed_value: 0` for an inverted [0,1] action, or use
physical width endpoints such as `0.08` and `0.0` for metres. Endpoints must be
finite and different. Both six-element poses must lie in [0,1]; they can hold
thumb opposition fixed or leave ring/pinky open for a pinch. The example's
all-open/all-closed poses are illustrative and require task-specific calibration.
No left/right mirroring is assumed beyond the separate configured poses.

Explicit `input: gripper` waits for a valid command and defaults to a 500 ms
watchdog. The auto-mode VLA profile specifies those same settings explicitly.
On expiry, the service requests zero velocity once and keeps publishing feedback;
a fresh valid sample resumes position control. Invalid packets do not refresh
the watchdog. Choose the timeout above the model's maximum expected command
gap, or explicitly set zero for indefinite targets. The SDK motion writes
return `void`, so neither target nor timeout writes provide an acknowledgement;
zero velocity cannot be guaranteed over a broken serial connection. Status
failures are logged and reads continue, but runtime port reconnection is not
implemented. Shutdown closes ports without issuing an open/close gesture.

## Validation

`test_bridge` covers native numeric compatibility, automatic one/six-entry
routing, format switching, explicit mode overrides, independent left/right
calibration, invalid input, inverted and width-based gripper mapping, endpoint
clipping, configuration validation, timeouts/resume, time-mode scheduling,
and concurrent command snapshots while switching formats.
`test_discovery` substitutes the SDK boundary to exercise first-probe loss,
failed opens, repeated metadata/unit-readback failures, reversed port order,
retaining one hand while finding the other, shared-port ownership, Revo1 and
wrong-side rejection, cancellation, cleanup, and SDK command dispatch.
`sdk_updater` uses offline ZIP fixtures to test both Linux architecture names
and aliases, same-version repairs, failed downloads/validation, installation
rollback, argument/platform rejection and concurrent-updater locking. It skips
if downloader dependencies are unavailable; it never changes the repo's SDK.

These tests do not move hardware. Run them without Unitree SDK2 using:

```bash
cmake -S . -B build/core-tests -DBUILD_SERVICE=OFF
cmake --build build/core-tests -j4
ctest --test-dir build/core-tests --output-on-failure
```

On the G1, verify repeated cold starts with both hands and confirm the logged
side/port bindings. To test discovery without starting DDS or sending motion
commands:

```bash
./bin/brainco_hand_server -c config/dual_revo2.yaml --detect-only
```

This still configures and verifies normalized units on accepted devices. Then
validate native six-finger commands and the configured gripper poses.
