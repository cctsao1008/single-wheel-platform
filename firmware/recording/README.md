# Recording

Firmware-owned binary recording contracts live here. They encode production semantic observations and profiling evidence for transport, capture, replay, and host decoding without changing the ownership of the underlying Plant, Control, or Supervisor semantics.

- `observation-record/` encodes `RawObservation` evidence.
- `runtime-observation-record/` encodes canonical runtime observations.
- `control-profile-record/` encodes control-path timing/profile evidence.
