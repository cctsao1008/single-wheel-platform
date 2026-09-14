# Console Information Architecture

The browser console is a presentation surface over already-produced evidence. It supports two explicit dialects without merging their semantics:

```text
simulator-neutral-v2              production-semantic-v1
plant/common observables          #17 nested closed-loop evidence
          |                                  |
          +----------------+-----------------+
                           |
                   presentation adapter
                           |
                   Single console UI
```

```text
+-------------------------------------------------------------------+
| Header: project identity | trace status | dialect | load trace    |
+----------+--------------------------------------+-----------------+
|          |                                      |                 |
| Sidebar  | Model viewport                       | Inspector       |
|          |                                      |                 |
| setup    | side: pitch / forward motion        | evidence        |
| sim      | front: roll / reaction phase        | displayed state |
| state    | synchronized trends                  | truth/estimate  |
| inputs   |                                      | reference       |
| evidence | split / side / front                | runtime         |
| trends   |                                      | applied input   |
| contract |                                      |                 |
+----------+--------------------------------------+-----------------+
| Transport: restart | play/pause | speed | scrub | time | log     |
+-------------------------------------------------------------------+
```

## Responsibility boundary

| Surface | Responsibility | Explicit non-responsibility |
| --- | --- | --- |
| Header | identify loaded evidence and dialect | infer missing backend/runtime semantics |
| Sidebar | navigate presentation surfaces and select model view | configure controller/firmware |
| Model viewport | project carried motion evidence into a drawing | integrate physics |
| Trend panel | project recorded samples and explicit semantic transitions over time | smooth, resample, estimate, integrate, or create runtime state |
| Inspector | preserve truth / estimate / reference / authority provenance | synthesize missing state |
| Transport | replay and scrub recorded evidence | alter simulation time or rerun a backend |
| Event log | report local validation/playback and recorded semantic transitions | act as a production/runtime logger |

## Dialect boundary

The flat simulator-neutral trace carries plant state and applied torque only. It does not carry estimator state, controller reference, runtime authority, or saturation provenance. Those remain visibly unavailable in that dialect, including in the trend panel.

The nested `closed_loop_production_path` evidence already contains explicit truth, production estimate/reference, runtime authority, actuation, and applied torque. The UI may display those fields because the trace carries them explicitly.

The UI does not upgrade one dialect into the other. It uses a presentation adapter so provenance remains visible.

## Synchronized trend boundary

The trend panel consumes the same normalized records used by the model viewport and inspector:

```text
normalized record
   |      |       |
   |      |       +--> runtime transition markers
   |      +----------> torque series
   +-----------------> attitude truth/common + optional estimate
```

The cursor is synchronized with the existing recorded-sample index. Clicking a chart seeks to the nearest recorded sample; it does not interpolate a new sample or rerun a simulator.

Runtime markers are transition-only evidence. A marker is created when an explicit production-semantic field changes, such as operating state, authority, actuation, constraint state, or fault bits. The UI does not manufacture events between samples.

## Derived diagnostics

Truth-minus-estimate residuals are permitted only where both values exist explicitly in the loaded closed-loop evidence. They are labeled UI-derived diagnostics.

Reaction-wheel phase is a useful example of the boundary: Webots truth carries it, while the reduced production estimate omits that cyclic coordinate. The UI therefore shows the truth phase but does not fabricate an estimated phase.

This distinction is deliberate: a polished console is not permission to invent telemetry.