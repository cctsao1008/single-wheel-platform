# Support

Cross-domain implementation support used by production-domain crates lives here.

`support/` is not a fifth production architecture domain. The production architecture remains exactly Plant, Control, Supervisor, and Firmware.

- `dsp-kernel/` provides the canonical real-time DSP implementation boundary used by Control and Supervisor while preserving a deterministic host semantic emulator for verification.
