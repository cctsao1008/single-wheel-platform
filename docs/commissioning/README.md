# Commissioning Configuration

This directory contains physical/runtime configuration contracts and accepted commissioning results consumed by the platform.

```text
runtime_profile.md              active STM32F103 acquisition / transport profile
encoder_physical_evidence.md    non-actuating encoder scale/sign evidence contract
```

Commissioning values are configuration data: coordinate transforms, encoder scale/sign, actuator electrical polarity, battery transfer functions, and authority limits enter the runtime through explicit parameters rather than controller-local constants.