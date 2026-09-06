# Repository Layout

The repository is a Rust workspace organized by semantic ownership.

```text
crates/
  robot-domain/                  Robot state, generalized demand, actuator-domain types
  plant-model/                   Nonlinear / reduced balance plant contracts
  measurement-model/             Physical measurement equations
  dsp-kernel/                    Fixed-size Cortex-M numerical kernels
  state-estimator/               Estimator interface + fixed-gain linear observer
  ekf/                           Nonlinear covariance-based estimator
  estimator-input/               Evidenced observations -> estimator measurement vector
  state-feedback/                LQR / LQI execution
  control-runtime/               Estimator -> feedback -> actuator -> authority composition
  actuator-model/                Torque/command inverse model and saturation
  one-v2-electrical-output/      Authorized command -> ONE V2 electrical line encoding
  runtime-state/                 Operating state, timing health, output authority
  reference-assembly/            Installed hardware and board-channel-to-role mapping
  plant-observation/             Raw values, timestamps, quality, acquisition status
  sensor-calibration/            Device scaling and measured calibration
  frame-transform/               Sensor-frame -> body-frame rotation
  observation-record/            Binary observation / replay contract
  control-profile-record/        Shadow-control timing/profile record contract
  mpu6050/                       MPU6050 protocol and transfer functions
  software-i2c/                  embedded-hal software I2C implementation
  board-one-v2/                  PCB pins, timers, connectors, buses

firmware/
  stm32f103/                     Observation runtime and concrete peripheral ownership
  stm32f103-electrical-output/   TIM3_CH1/CH4 + PA4/PB11 authorized output sink
  live-shadow-stm32f103/         Non-actuating live control profiler
  control-footprint-stm32f103/   Non-actuating target linkage/footprint probe

parameters/
  reference-assembly.json        Provenance-bearing reference-assembly parameters

tools/
  model/                         Host model derivation
  control/                       Host controller / observer synthesis
  actuator/                      Actuator identification
  recording/                     Decode and deterministic replay
  wireless/                      BLE observation transport

docs/
  architecture/                  System contracts
  hardware/                      Platform and pin mapping
  commissioning/                 Runtime / physical commissioning definitions
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

```text
actuator-model
    physical effort <-> normalized command

one-v2-electrical-output
    authorized command -> board electrical encoding

stm32f103-electrical-output
    concrete TIM3 / GPIO mutation
```

Target firmware owns concrete STM32 peripherals and composes the runtime. Portable crates do not own MCU resources.

UART, BLE, OLED, storage, and host utilities are interfaces or transports; they do not own the system data model.
