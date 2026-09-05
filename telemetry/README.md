# Telemetry

Telemetry observes the control system without becoming part of the controller.

The preferred runtime model is a bounded binary trace / ring buffer in the time-critical path, with text, CSV, JSON, or plotting conversion performed outside that path or on the host.
