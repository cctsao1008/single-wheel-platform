# Tools

Host-side engineering tools live here. They consume explicit firmware data contracts rather than scraping implementation details.

```text
recording/    canonical raw-observation decode and deterministic replay
plotting/     signal and controller plots (future)
system_id/    plant characterization and model fitting (future)
```

Recording/replay is a first-class architecture path. A captured `RawObservation` stream must remain usable independently of UART timing and independently of the estimator/controller version that produced the recording.

Generated captures, CSV files, plots, and fitted artifacts are data and should not be committed as firmware source unless a specific reproducibility fixture is intentionally added.
