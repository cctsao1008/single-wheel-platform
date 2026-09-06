# Communications

`communications/` is the architectural home for reusable external communication modules and endpoint/protocol behavior.

Examples include an ECB02 BLE module integration or another external telemetry/command endpoint once its concrete hardware identity and reusable behavior are implemented.

Concrete MCU UART/DMA ownership belongs in `firmware/targets/`; board wiring belongs in `firmware/boards/`. Communication code does not own robot control policy.
