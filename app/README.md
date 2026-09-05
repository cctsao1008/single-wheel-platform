# Application Layer

`app/` owns system orchestration and application services.

Responsibilities include:

- initialization order,
- connecting platform data to the control pipeline,
- command and maintenance services,
- calibration workflow,
- configuration lifecycle,
- non-critical background service scheduling.

It must not embed MCU register access or controller mathematics.
