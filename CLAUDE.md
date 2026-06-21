# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP32 firmware (PlatformIO + Arduino framework) that physically drives the grill: vertical position (linear actuator), tilt/rotation (rotor motor), temperature sensing, manual control, and execution of multi-step cooking programs received over MQTT. Part of the larger GaztaindiGrill ecosystem — see `../CLAUDE.md` for how this fits with the API and web client, and for the MQTT topic source of truth.

## Commands

```bash
# Build (default env = esp32doit-devkit-v1, OTA over Ethernet)
pio run

# First-ever flash of a board (OTA endpoint doesn't exist yet): flash over USB
pio run -e usb -t upload

# Subsequent flashes: OTA over Ethernet (W5500), POSTs firmware.bin to the device's /update endpoint
pio run -e esp32doit-devkit-v1 -t upload
# equivalent to: pio run -t upload (default_envs)

# Serial monitor (115200 baud)
pio device monitor
```

There is no automated test suite (`test/` only contains the PlatformIO Unity placeholder `README`). Verification is manual: flash and observe serial output / MQTT traffic, or point the web client at `GaztaindiGrill-Shadow` instead of real hardware to test protocol-level changes without flashing.

The OTA upload target posts to a hardcoded device IP in `platformio.ini` (`http://192.168.1.100:3232/update`) — update that if the device's static IP changes (see `GRILL_Modules/GRILL_config.h`).

## Architecture

Full design write-up lives in [ARCHITECTURE.md](ARCHITECTURE.md) — read it before modifying control flow, MQTT handling, or program execution. Summary of the class responsibilities (all in `lib/Grill/`):

- **`GaztaindiGrill.cpp`** (`src/`) — entry point (`setup()`/`loop()`), owns the Ethernet/MQTT connections, dispatches incoming MQTT messages to the right `Grill` instance.
- **`GrillSystem`** — owns the 2 `Grill` instances (`GrillConstants::NUM_GRILLS`).
- **`Grill`** — facade for a single grill; routes MQTT commands to the right manager.
- **`ProgramManager`** — state machine (`ProgramState`/`StepState`) that drives step-by-step program execution; program state lives in RAM only and does **not** survive a reboot (deliberate tradeoff to avoid flash wear — see ARCHITECTURE.md §4 for the FRAM-based plan if this ever needs to change).
- **`MovementManager`** — vertical actuator + rotation motor control.
- **`GrillMQTT`** — wrapper centralizing publish/subscribe/topic-parsing logic.
- **`HardwareManager`, `GrillSensor`, `StatusLed`** — low-level hardware abstraction.
- **`DualModeCoordinator`** — coordinates the two grills when running in "dual" mode (vs "single").

Key cross-cutting flows (detailed in ARCHITECTURE.md):

- **Multi-user sync**: the ESP32 periodically publishes a lightweight progress status; a newly-connected client that sees an unfamiliar `programId` requests the full program JSON on demand rather than the API, since the ESP32 holds the actually-running version in RAM.
- **Disconnection handling**: uses MQTT Last Will and Testament (`grill/connection` → `offline`) so clients can detect a dead grill instead of showing stale "running" state.

`GrillConstants.h` is the single source of truth for every MQTT topic, payload string, and tunable constant (timeouts, margins, `NUM_GRILLS`, `MAX_PROGRAM_STEPS`, etc.) — check it before touching anything protocol-related, and keep `GaztaindiGrill-NextJS/src/constants/mqtt.ts` in sync if you add or rename a topic.

`lib/` also vendors third-party libraries (Adafruit MAX31855/MAX31865, ArduinoJson, PubSubClient, CytronMotorDriver, etc.) alongside the project's own code — only `Grill/`, `GRILL_Modules/`, `DeviceEncoder/`, `DeviceRotorDrive/`, `EthernetNTP/`, `EthernetOTA/`, and `SerialTelnet/` are project-owned.
