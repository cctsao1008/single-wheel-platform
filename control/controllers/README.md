# Controllers

Controller implementations are plant-specific algorithms behind control-domain interfaces.

Initial decomposition:

- **Roll:** attitude regulation plus reaction-wheel speed regulation.
- **Pitch:** attitude regulation plus drive-wheel speed regulation.
- **Yaw:** spin-actuator command path, with closed-loop policy added only when sensing and plant behavior support it.

The architecture also allows future coupled state-feedback controllers without changing platform I/O contracts.
