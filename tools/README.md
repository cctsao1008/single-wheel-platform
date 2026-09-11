# Tools

Host-side engineering tools consume explicit production data and model contracts.

```text
model/          symbolic plant derivation and independent numeric reference/correlation
recording/      decode `RecordedObservation` streams and replay them deterministically
commissioning/  turn bounded non-actuating bench experiments into reproducible evidence candidates
wireless/       capture and observe the ECB02S2 BLE record stream
sitl/           deterministic host-side software-in-the-loop execution and evidence
simulation/     backend-neutral experiment/evidence contracts for independent simulators
```

Generated captures, CSV files, plots, fitted artifacts, reference traces, commissioning evidence, SITL output, and simulator traces are runtime/engineering data rather than firmware source.