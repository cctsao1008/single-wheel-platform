# Control Safety

This directory owns control-domain validity and output qualification.

Typical checks include:

- state freshness and validity,
- finite numeric values,
- roll / pitch operating envelopes,
- wheel overspeed,
- actuator saturation,
- command slew and reversal limits,
- battery and timing conditions.

Control safety produces qualified requests; physical motor ownership remains a separate system responsibility.
