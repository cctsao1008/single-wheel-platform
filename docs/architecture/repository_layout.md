# Repository Layout

The repository is a Rust workspace organized by semantic ownership.

```text
crates/
  robot-domain/          Robot state, generalized demand, actuator-domain types
  reference-assembly/    Installed hardware and board-channel-to-role mapping
  plant-observation/     Raw values, timestamps, quality, acquisition status
  sensor-calibration/    Device scaling and measured sensor-frame calibration
  frame-transform/       Sensor-frame to body-frame rotation
  runtime-state/         Operating state and actuator authority
  observation-record/    Binary record / replay contract
  mpu6050/               MPU6050 protocol and transfer functions
  software-i2c/          embedded-hal software I2C implementation
  board-one-v2/          PCB pins, timers, connectors, buses

firmware/
  stm32f103/             STM32F103 RTIC runtime and concrete peripheral ownership

tools/
  recording/             Host decode and deterministic replay

docs/
  architecture/          System contracts
  hardware/              Platform and pin mapping
  commissioning/         Runtime/physical configuration results
```

## Ownership boundaries

```text
board-one-v2
    PCB capability

reference-assembly
    installed hardware

robot-domain
    robot semantics
```

```text
mpu6050
    register protocol + device transfer functions

sensor-calibration
    measured sensor correction

frame-transform
    mechanical coordinate mapping
```

```text
plant-observation
    acquisition evidence

observation-record
    persistent/replay representation

runtime-state
    operating authority
```

`firmware/stm32f103` owns concrete STM32 peripherals and composes the target runtime. Portable crates do not own MCU resources.

UART, BLE, OLED, storage, and host utilities are interfaces or transports; they do not own the system data model.
